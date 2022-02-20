#!/usr/bin/env python3
#
# This script uses the ViSP tracker to provide the robot
# localization.
#
# We make the assumption that the position of a particular object in
# the world frame is known, therefore it is possible to deduce the
# robot position knowing the object position w.r.t. the camera and the
# camera position w.r.t. the base link.
#
# In the ROS framework, the 'map' frame is dedicated to this kind of
# global localization where the robot pose is not required to be
# continuous. Therefore this script streams this tf transformation.

from __future__ import print_function

import numpy as np
import roslib; roslib.load_manifest('visp_tracker')
import rospy

import tf
from geometry_msgs.msg import TransformStamped

from tf.transformations import quaternion_from_matrix, \
    translation_from_matrix, quaternion_from_matrix

rospy.init_node('localizer')

def callback(cMo_msg):
    global oMw, blMc, tl, tr

    # Get object position w.r.t. the camera frame as an homogeneous
    # matrix.
    cMo = np.matrix(tr.fromTranslationRotation
                    ((cMo_msg.transform.translation.x,
                      cMo_msg.transform.translation.y,
                      cMo_msg.transform.translation.z),
                     (cMo_msg.transform.rotation.x,
                      cMo_msg.transform.rotation.y,
                      cMo_msg.transform.rotation.z,
                      cMo_msg.transform.rotation.w)))

    # Retrieve base link position w.r.t. the camera.
    try:
        t = tl.lookupTransform(baseLinkFrameId, cMo_msg.header.frame_id,
                               rospy.Time(0))
    except:
        rospy.logwarn("failed to retrieve camera position w.r.t. base_link")
        return

    blMc = np.matrix(tr.fromTranslationRotation(t[0], t[1]))

    # Compute the base link position w.r.t the world frame.
    blMw = blMc * cMo * oMw

    blMw_t = translation_from_matrix(blMw)
    blMw_q = quaternion_from_matrix(blMw)

    # Send computed position to tf.
    br = tf.TransformBroadcaster()

    br.sendTransform(blMw_t, blMw_q,
                     cMo_msg.header.stamp,
                     baseLinkFrameId,
                     mapFrameId)
    #rospy.loginfo("map position sent")

# Frame ids.
baseLinkFrameId = rospy.get_param('~base_link_frame_id', '/base_link')
mapFrameId = rospy.get_param('~map_frame_id', '/map')

tr = tf.TransformerROS()
tl = tf.TransformListener()

# Object position w.r.t. world frame.
wMo = np.matrix(np.identity(4))
wMo_t = [0., 0., 0.]
wMo_q = [0., 0., 0., 1.]
wMo_t[0] = rospy.get_param('~object_translation_x', 0.)
wMo_t[1] = rospy.get_param('~object_translation_y', 0.)
wMo_t[2] = rospy.get_param('~object_translation_z', 0.)
wMo_q[0] = rospy.get_param('~object_translation_qx', 0.)
wMo_q[1] = rospy.get_param('~object_translation_qy', 0.)
wMo_q[2] = rospy.get_param('~object_translation_qz', 0.)
wMo_q[3] = rospy.get_param('~object_translation_qw', 1.)
wMo = np.matrix(tr.fromTranslationRotation(wMo_t, wMo_q))
oMw = np.linalg.inv(wMo)

blMc = np.matrix(np.identity(4))


rospy.Subscriber("object_position", TransformStamped, callback)
rospy.loginfo("start streaming map position")
rospy.spin()
