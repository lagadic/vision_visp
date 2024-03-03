#!/usr/bin/env python3
# ROS2
import rclpy
from rclpy.node import Node
from visp_megapose.srv import Init, Track, Render
import transforms3d

import os
import json
import megapose_server
megapose_server_install_dir = os.path.dirname(megapose_server.__file__)
variables_file = os.path.join(megapose_server_install_dir, 'megapose_variables_final.json')
with open(variables_file, 'r') as f:
    json_vars = json.load(f)
    print('Loaded megapose variables', json_vars)
    os.environ['MEGAPOSE_DIR'] = json_vars['megapose_dir']
    os.environ['MEGAPOSE_DATA_DIR'] = json_vars['megapose_data_dir']

if 'HOME' not in os.environ: # Home is always required by megapose but is not always set
    if os.name == 'nt':
      if 'HOMEPATH' in os.environ:
        os.environ['HOME'] = os.environ['HOMEPATH']
      elif 'HOMEDIR' in os.environ:
        os.environ['HOME'] = os.environ['HOMEDIR']
      else:
        os.environ['HOME'] = '.'
    else:
      os.environ['HOME'] = '.'

# 3rd party
import numpy as np
import argparse
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Union
# from PIL import Image
# import socket
# import struct
# import io
# import sys
# import traceback
# from operator import itemgetter
import pandas as pd
import torch
import torch.fx as fx
import torch.nn as nn
from torch.fx.experimental.optimization import optimize_for_inference, fuse
import time
# MegaPose
from megapose.datasets.object_dataset import RigidObject, RigidObjectDataset
from megapose.datasets.scene_dataset import CameraData, ObjectData
from megapose.inference.types import (
    DetectionsType,
    ObservationTensor,
    PoseEstimatesType,
)
from megapose.inference.utils import make_detections_from_object_data
from megapose.lib3d.transform import Transform
from megapose.panda3d_renderer import Panda3dLightData
from megapose.panda3d_renderer.panda3d_scene_renderer import Panda3dSceneRenderer
from megapose.utils.conversion import convert_scene_observation_to_panda3d
from megapose.utils.load_model import NAMED_MODELS, load_named_model

# Megapose server
from megapose_server.network_utils import *
from megapose_server.server_operations import ServerMessage

megapose_models = {
        'RGB': ('megapose-1.0-RGB', False),
        'RGBD': ('megapose-1.0-RGBD', True),
        'RGB-multi-hypothesis': ('megapose-1.0-RGB-multi-hypothesis', False),
        'RGBD-multi-hypothesis': ('megapose-1.0-RGB-multi-hypothesis-icp', True),
    }
camera_data = {
        'K': np.asarray([
            [700, 0.0, 320],
            [0.0, 700, 240],
            [0.0, 0.0, 1.0]
        ]),
        'h': 480,
        'w': 640
}
def make_object_dataset(meshes_dir: Path) -> RigidObjectDataset:
    rigid_objects = []
    mesh_units = "m"
    object_dirs = meshes_dir.iterdir()
    for object_dir in object_dirs:
        label = object_dir.name
        mesh_path = None
        for fn in object_dir.glob("*"):
            if fn.suffix in {".obj", ".ply", ".glb", ".gltf"}:
                assert not mesh_path, f"there are multiple meshes in the {label} directory"
                mesh_path = fn
        assert mesh_path, f"couldnt find the mesh for {label}"
        rigid_objects.append(RigidObject(label=label, mesh_path=mesh_path, mesh_units=mesh_units))
    rigid_object_dataset = RigidObjectDataset(rigid_objects)
    return rigid_object_dataset

class MegaPoseServer(Node):
    def __init__(self, image_batch_size=256, warmup=True):
        super().__init__('MegaPoseServer')
        print('Parameters:')
        self.declare_parameter('mesh_dir', 'visp_megapose/data/models')
        mesh_dir = self.get_parameter('mesh_dir').get_parameter_value().string_value
        mesh_dir = Path(mesh_dir).absolute()
        assert mesh_dir.exists(), 'Mesh directory does not exist, cannot start server' 
        self.declare_parameter('megapose_models', 'RGB')
        model_name = self.get_parameter('megapose_models').get_parameter_value().string_value
        model_name = megapose_models[model_name][0]
        self.declare_parameter('num_workers', 4)
        num_workers = self.get_parameter('num_workers').get_parameter_value().integer_value
        self.declare_parameter('optimize', False)
        optimize = self.get_parameter('optimize').get_parameter_value().bool_value
        
        self.srv_initial_pose = self.create_service(Init, 'initial_pose', self.InitPoseCallback)
        self.srv_track_pose = self.create_service(Track, 'track_pose', self.TrackPoseCallback)
        self.srv_render_object = self.create_service(Render, 'render_object', self.RenderObjectCallback)
        
        self.num_workers = num_workers
        self.object_dataset: RigidObjectDataset = make_object_dataset(mesh_dir)
        model_tuple = self._load_model(model_name)
        self.model_info = model_tuple[0]
        self.model = model_tuple[1]
        self.model.eval()
        self.model.bsz_images = image_batch_size
        self.camera_data = self._make_camera_data(camera_data)
        self.renderer = Panda3dSceneRenderer(self.object_dataset)
        torch.backends.cudnn.benchmark = True
        torch.backends.cudnn.deterministic = False
        self.optimize = optimize
        self.warmup = warmup
        if self.optimize:
            print('Optimizing Pytorch models...')
            class Optimized(nn.Module):
                def __init__(self, m: nn.Module, inp):
                    super().__init__()
                    self.m = m.eval()
                    self.m = fuse(self.m, inplace=False)
                    self.m = torch.jit.trace(self.m, torch.rand(inp).cuda())
                    self.m = torch.jit.freeze(self.m)

                def forward(self, x):
                    return self.m(x).float()

            h, w = self.camera_data.resolution
            self.model.coarse_model.backbone = Optimized(self.model.coarse_model.backbone, (1, 9, h, w))
            self.model.refiner_model.backbone = Optimized(self.model.refiner_model.backbone, (1, 32 if self.model_info['requires_depth'] else 27, h, w))

        if self.warmup:
            self.get_logger().info('Warming up models...')
            h, w = self.camera_data.resolution
            labels = self.object_dataset.label_to_objects.keys()
            observation = self._make_observation_tensor(np.random.randint(0, 255, (h, w, 3), dtype=np.uint8),
                                                        np.random.rand(h, w).astype(np.float32) if self.model_info['requires_depth'] else None).cuda()
            detections = self._make_detections(labels, np.asarray([[0, 0, w//2, h//2] for _ in range(len(labels))], dtype=np.float32)).cuda()
            self.model.run_inference_pipeline(observation, detections, **self.model_info['inference_parameters'])
        self.get_logger().info('waiting for requests...')
    def _load_model(self, model_name):
        return NAMED_MODELS[model_name], load_named_model(model_name, self.object_dataset, n_workers=self.num_workers).cuda()
    
    def _make_camera_data(self, camera_data: Dict) -> CameraData:
        '''
        Create a camera representation that is understandable by megapose.
        camera_data: A dict containing the keys K, h, w
        K is the 3x3 intrinsics matrix
        h and w are the input image resolution.

        Returns a CameraData object, to be given to megapose.
        '''
        c = CameraData()
        c.K = camera_data['K']
        c.resolution = (camera_data['h'], camera_data['w'])
        print(c)
        c.z_near = 0.001
        c.z_far = 100000
        return c
    
    def InitPoseCallback(self, request, response):
        # request: object_name, topleft_i, topleft_j, bottomright_i, bottomright_j, image, camera_info
        # response: pose, scores, bounding_boxes
        depth = None
        camera_data = {
            'K': np.asarray([
                [request.camera_info.k[0], request.camera_info.k[1], request.camera_info.k[2]],
                [request.camera_info.k[3], request.camera_info.k[4], request.camera_info.k[5]],
                [request.camera_info.k[6], request.camera_info.k[7], request.camera_info.k[8]]
            ]),
            'h': request.camera_info.height,
            'w': request.camera_info.width
        }
        self.camera_data = self._make_camera_data(camera_data)
        print(self.camera_data)
        img = np.array(request.image.data).reshape((request.camera_info.height, request.camera_info.width, 3))
        object_name = [request.object_name]
        detections = [[request.topleft_j,request.topleft_i , request.bottomright_j, request.bottomright_i ]]
        detections = self._make_detections(object_name, detections).cuda()
        observation = self._make_observation_tensor(img, depth).cuda()
        inference_params = self.model_info['inference_parameters'].copy()
        output, extra_data = self.model.run_inference_pipeline(
            observation, detections=detections, **inference_params, coarse_estimates=None
        )
        poses = output.poses.cpu().numpy()
        poses = poses.reshape(len(poses), 4, 4)
        confidence = output.infos['pose_score'].to_numpy()

        bounding_boxes = extra_data['scoring']['preds'].tensors['boxes_rend'].cpu().numpy().reshape(-1, 4)
        bounding_boxes = bounding_boxes.tolist()

        response.pose.translation.x = float(poses[0][0, 3])
        response.pose.translation.y = float(poses[0][1, 3])
        response.pose.translation.z = float(poses[0][2, 3])
        rotation = poses[0][0:3, 0:3]
        rotation = transforms3d.quaternions.mat2quat(rotation)
        response.pose.rotation.x = rotation[1]
        response.pose.rotation.y = rotation[2]
        response.pose.rotation.z = rotation[3]
        response.pose.rotation.w = rotation[0]
        response.confidence = float(confidence[0])
        return response
    
    def TrackPoseCallback(self, request, response):
        # request: object_name, init_pose, refiner_iterations, image, camera_info
        # response: pose
        t = time.time()
        depth = None
        img = np.array(request.image.data).reshape((request.camera_info.height, request.camera_info.width, 3))
        object_name = [request.object_name]

        cTos_np = np.eye(4)
        cTos_np[0:3, 3] = [request.init_pose.translation.x, request.init_pose.translation.y, request.init_pose.translation.z]
        cTos_np[0:3, 0:3] = transforms3d.quaternions.quat2mat([request.init_pose.rotation.w, request.init_pose.rotation.x, request.init_pose.rotation.y, request.init_pose.rotation.z])
        cTos_np = cTos_np.reshape(1, 4, 4)
        tensor = torch.from_numpy(cTos_np).float().cuda()
        infos = pd.DataFrame.from_dict({
            'label': object_name,
            'batch_im_id': [0 for _ in range(len(cTos_np))],
            'instance_id': [i for i in range(len(cTos_np))]
        })
        coarse_estimates = PoseEstimatesType(infos, poses=tensor)

        detections = None
        observation = self._make_observation_tensor(img, depth).cuda()
        inference_params = self.model_info['inference_parameters'].copy()
        inference_params['n_refiner_iterations'] = request.refiner_iterations
        output, extra_data = self.model.run_inference_pipeline(
            observation, detections=detections, **inference_params, coarse_estimates=coarse_estimates
        )
        poses = output.poses.cpu().numpy()
        poses = poses.reshape(len(poses), 4, 4)
        confidence = output.infos['pose_score'].to_numpy()
        bounding_boxes = extra_data['scoring']['preds'].tensors['boxes_rend'].cpu().numpy().reshape(-1, 4)
        bounding_boxes = bounding_boxes.tolist()

        response.pose.translation.x = float(poses[0][0, 3])
        response.pose.translation.y = float(poses[0][1, 3])
        response.pose.translation.z = float(poses[0][2, 3])
        rotation = poses[0][0:3, 0:3]
        rotation = transforms3d.quaternions.mat2quat(rotation)
        response.pose.rotation.x = rotation[1]
        response.pose.rotation.y = rotation[2]
        response.pose.rotation.z = rotation[3]
        response.pose.rotation.w = rotation[0]
        response.confidence = float(confidence[0])

        return response
    def RenderObjectCallback(self, request, response):
        # request: object_name, pose, camera_info
        # response: image
        labels = [request.object_name]
        # print("labels", labels)
        poses = np.eye(4)
        poses[0:3, 3] = [request.pose.translation.x, request.pose.translation.y, request.pose.translation.z]
        poses[0:3, 0:3] = transforms3d.quaternions.quat2mat([request.pose.rotation.w, request.pose.rotation.x, request.pose.rotation.y, request.pose.rotation.z])
        poses = poses.reshape(1, 4, 4)
        # print("poses", poses)
        camera_data = CameraData()
        camera_data.K = self.camera_data.K
        camera_data.resolution = self.camera_data.resolution
        camera_data.TWC = Transform(np.eye(4))
        
        object_datas = []
        for label, pose in zip(labels, poses):
            object_datas.append(ObjectData(label=label, TWO=Transform(pose)))
        camera_data, object_datas = convert_scene_observation_to_panda3d(camera_data, object_datas)
        light_datas = [
            Panda3dLightData(
                light_type="ambient",
                color=((1.0, 1.0, 1.0, 1)),
            ),
        ]
        renderings = self.renderer.render_scene(
            object_datas,
            [camera_data],
            light_datas,
            render_depth=False,
            render_binary_mask=False,
            render_normals=False,
            copy_arrays=True,
        )[0]
        img = renderings.rgb
        # change to ROS2 image, reshape to 1D array
        img = np.uint8(img)
        img = img.reshape(1, -1).tolist()[0]
        
        response.image.header.stamp = self.get_clock().now().to_msg()
        response.image.height = renderings.rgb.shape[0]
        response.image.width = renderings.rgb.shape[1]
        response.image.encoding = 'rgb8'
        response.image.is_bigendian = 0
        response.image.step = 3 * renderings.rgb.shape[1]
        response.image.data = img

        return response
    

    def _make_detections(self, labels, detections):
        result = []
        for label, detection in zip(labels, detections):
            o = ObjectData(label)
            o.bbox_modal = detection
            result.append(o)

        return make_detections_from_object_data(result)
    
    def _make_observation_tensor(self, image: np.ndarray, depth: Optional[np.ndarray]=None) -> ObservationTensor:
        '''
        Create an observation tensor from an image and a potential depth image
        '''
        return ObservationTensor.from_numpy(image, depth, self.camera_data.K)

            
def main():
    rclpy.init()
    server = MegaPoseServer()
    rclpy.spin(server)


if __name__ == '__main__':
    main()
