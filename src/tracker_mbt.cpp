#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <ros/param.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>

#include <visp_tracker/CameraParameters.h>
#include <visp_tracker/TrackingResult.h>
#include <visp_tracker/Init.h>

#include <boost/bind.hpp>
#include <visp/vpImage.h>
#include <visp/vpImageConvert.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpMbEdgeTracker.h>

#include "conversion.hh"
#include "callbacks.hh"
#include "file.hh"

// TODO:
// - add a topic allowing to suggest an estimation of the cMo
// - handle automatic reset when tracking is lost.

typedef vpImage<unsigned char> image_t;

typedef boost::function<bool (visp_tracker::Init::Request&,
			      visp_tracker::Init::Response& res)>
init_callback_t;

typedef boost::function<bool (visp_tracker::CameraParameters::Request&,
			      visp_tracker::CameraParameters::Response& res)>
camera_parameters_callback_t;

enum State
  {
    WAITING_FOR_INITIALIZATION,
    TRACKING,
    LOST
  };

bool initCallback(State& state,
		  vpMbEdgeTracker& tracker,
		  image_t& image,
		  std::string& model_path,
		  std::string& model_name,
		  std::string& model_configuration,
		  visp_tracker::Init::Request& req,
		  visp_tracker::Init::Response& res)
{
  ROS_INFO("Initialization request received.");

  // Update the parameters.
  ros::param::set("model_path", req.model_path.data);
  ros::param::set("model_name", req.model_name.data);
  ros::param::set("model_configuration", req.model_configuration.data);

  ros::param::set("vpme_mask_size", (int)req.moving_edge.mask_size);
  ros::param::set("vpme_range", (int)req.moving_edge.range);
  ros::param::set("vpme_threshold", req.moving_edge.threshold);
  ros::param::set("vpme_mu1", req.moving_edge.mu1);
  ros::param::set("vpme_mu2", req.moving_edge.mu2);


  model_path = req.model_path.data;
  model_name = req.model_name.data;
  model_configuration = req.model_configuration.data;

  // Load moving edges.
  vpMe moving_edge;
  moving_edge.mask_size = req.moving_edge.mask_size;
  moving_edge.range = req.moving_edge.range;
  moving_edge.threshold = req.moving_edge.threshold;
  moving_edge.mu1 = req.moving_edge.mu1;
  moving_edge.mu2 = req.moving_edge.mu2;

  // Reset the tracker and the node state.
  tracker.resetTracker();
  state = WAITING_FOR_INITIALIZATION;

  tracker.setMovingEdge(moving_edge);

  // Load the model.
  try
    {
      ROS_DEBUG_STREAM("Trying to load the model: " << model_path.c_str());
      tracker.loadModel
	(getModelFileFromModelName
	 (model_name, model_path).external_file_string().c_str());
    }
  catch(...)
    {
      //FIXME: revert model_path, etc. here.
      ROS_ERROR_STREAM("Failed to load the model: " << model_path.c_str());
      res.initialization_succeed = false;
      return true;
    }
  ROS_DEBUG("Model has been successfully loaded.");

  // Load the initial cMo.
  vpHomogeneousMatrix cMo;
  transformToVpHomogeneousMatrix(cMo, req.initial_cMo);

  // Try to initialize the tracker.
  ROS_INFO_STREAM("Initializing tracker with cMo:\n" << cMo);
  res.initialization_succeed = true;
  state = TRACKING;
  try
    {
      tracker.init(image, cMo);
      ROS_INFO("Tracker successfully initialized.");

      moving_edge.print();
    }
  catch(...)
    {
      state = WAITING_FOR_INITIALIZATION;
      res.initialization_succeed = false;
      ROS_ERROR("Tracker initialization has failed.");
    }
  return true;
}

bool cameraParametersCallback(const vpCameraParameters& cam,
			      visp_tracker::CameraParameters::Request& req,
			      visp_tracker::CameraParameters::Response& res)
{
  ROS_INFO("Camera parameters request received.");
  res.px = cam.get_px();
  res.py = cam.get_py();
  res.u0 = cam.get_u0();
  res.v0 = cam.get_v0();
  return true;
}

init_callback_t bindInitCallback(State& state,
				 vpMbEdgeTracker& tracker,
				 image_t& image,
				 std::string& model_path,
				 std::string& model_name,
				 std::string& model_configuration)
{
  return boost::bind(initCallback,
		     boost::ref(state),
		     boost::ref(tracker),
		     boost::ref(image),
		     boost::ref(model_path),
		     boost::ref(model_name),
		     boost::ref(model_configuration),
		     _1, _2);
}

camera_parameters_callback_t
bindCameraParametersCallback(const vpCameraParameters& cam)
{
  return boost::bind(cameraParametersCallback, cam, _1, _2);
}

int main(int argc, char **argv)
{
  State state = WAITING_FOR_INITIALIZATION;
  std::string image_topic;
  std::string model_path;
  std::string model_name;
  std::string model_configuration;
  vpMe moving_edge;
  double px;
  double py;
  double u0;
  double v0;

  image_t I;

  ros::init(argc, argv, "tracker_mbt");

  ros::NodeHandle n("tracker_mbt");
  image_transport::ImageTransport it(n);

  // Parameters.
  ros::param::param<std::string>("~image", image_topic, "/camera/image_raw");
  ros::param::param<std::string>("model_path", model_path, "");
  ros::param::param<std::string>("model_name", model_name, "");
  ros::param::param<std::string>("model_configuration",
				 model_configuration, "default");

  //FIXME: default parameters are just wrong.
  // - replace by real camera parameters of the rectified camera.
  ros::param::param("~px", px, 391.1807703);
  ros::param::param("~py", py, 390.9830219);
  ros::param::param("~u0", u0, 248.987914);
  ros::param::param("~v0", v0, 245.2349518);

  ros::param::param("vpme_mask_size", moving_edge.mask_size, 7);
  ros::param::param("vpme_range", moving_edge.range, 8);
  ros::param::param("vpme_threshold", moving_edge.threshold, 100.);
  ros::param::param("vpme_mu1", moving_edge.mu1, 0.5);
  ros::param::param("vpme_mu2", moving_edge.mu2, 0.5);

  // Result publisher.
  ros::Publisher result_pub =
    n.advertise<visp_tracker::TrackingResult>("result", 1000);

  // Output publisher.
  image_transport::Publisher output_pub = it.advertise("output", 1);

  // Camera subscriber.
  image_transport::CameraSubscriber sub =
    it.subscribeCamera(image_topic, 100, bindImageCallback(I));

  // Initialization.
  ros::Rate loop_rate(10);

  vpMbEdgeTracker tracker;
  tracker.setMovingEdge(moving_edge);

  // Tracker initialization.
  vpCameraParameters cam(px, py, u0, v0);
  tracker.setCameraParameters(cam);
  tracker.setDisplayMovingEdges(false);

  ROS_INFO_STREAM(cam);

  // Service declaration.
  ros::ServiceServer service = n.advertiseService
    ("init_tracker",
     bindInitCallback(state, tracker, I,
		      model_path, model_name, model_configuration));

  ros::ServiceServer serviceCameraParameters = n.advertiseService
    ("camera_parameters",
     bindCameraParametersCallback(cam));


  // Wait for the image to be initialized.
  while (!I.getWidth() || !I.getHeight())
    {
      ros::spinOnce();
      loop_rate.sleep();
    }

  // Main loop.
  while (ros::ok())
    {
      vpHomogeneousMatrix cMo;
      cMo.eye();

      if (state == TRACKING)
	try
	  {
	    tracker.track(I);
	    ROS_DEBUG("Tracking ok.");
	    tracker.getPose(cMo);
	  }
	catch(...)
	  {
	    ROS_WARN("Tracking lost.");
	    state = LOST;
	  }

      // Publish the tracking result.
      visp_tracker::TrackingResult result;
      result.is_tracking = state == TRACKING;
      if (state == TRACKING)
	{
	  vpHomogeneousMatrixToTransform(result.cMo.transform, cMo);
	  //FIXME: to be done result.cMo.header
	  //FIXME: to be done result.cMo.child_frame_id
	}
      result_pub.publish(result);

      ros::spinOnce();
      loop_rate.sleep();
    }

  return 0;
}
