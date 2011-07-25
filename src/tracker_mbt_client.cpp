#include <cstdlib>
#include <fstream>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <ros/param.h>
#include <visp_tracker/Init.h>

#include <visp/vpMbEdgeTracker.h>
#include <visp/vpDisplayX.h>

#include "conversion.hh"
#include "callbacks.hh"
#include "file.hh"

typedef vpImage<unsigned char> image_t;

bool fileExists(const std::string& file)
{
  std::ifstream istream (file.c_str());
  return istream;
}

int main(int argc, char **argv)
{
  std::string image_topic;

  std::string model_path;
  std::string model_name;
  std::string model_configuration;

  std::string init_service;

  image_t I;

  ros::init(argc, argv, "tracker_mbt_client");

  ros::NodeHandle n;
  image_transport::ImageTransport it(n);

  // Parameters.
  ros::param::param<std::string>("~image", image_topic, "/camera/image_raw");

  ros::param::param<std::string>("~model_path", model_path, "");
  ros::param::param<std::string>("~model_name", model_name, "");
  ros::param::param<std::string>("~model_configuration",
				 model_configuration, "default");

  ros::param::param<std::string>("~init_service",
				 init_service, "/tracker_mbt/init_tracker");

  // Camera subscriber.
  image_transport::CameraSubscriber sub =
    it.subscribeCamera(image_topic, 100, bindImageCallback(I));

  vpMbEdgeTracker tracker;

  // Model loading.
  std::string vrml_path =
    getModelFileFromModelName(model_name, model_path).external_file_string();
  std::string init_path =
    getInitFileFromModelName(model_name, model_path).external_file_string();
  std::string xml_path =
    getConfigurationFileFromModelName
    (model_name, model_path).external_file_string();

  ROS_DEBUG("VRML file: %s", vrml_path.c_str());
  ROS_DEBUG("Init file: %s.init", init_path.c_str());
  ROS_DEBUG("Configuration file: %s", xml_path.c_str());

  try
    {
      ROS_DEBUG("Trying to load the model `%s'.", vrml_path.c_str());
      tracker.loadModel(model_path.c_str());
    }
  catch(...)
    {
      ROS_ERROR("Failed to load the model `%s'.", vrml_path.c_str());
      return 1;
    }
  ROS_DEBUG("Model has been successfully loaded.");

  if (!fileExists(init_path + ".init"))
    {
      ROS_ERROR("Failed to load initialization points `%s.init'.",
		init_path.c_str());
      return 1;
    }

  // Tracker initialization.
  //FIXME: replace by real camera parameters of the rectified camera.
  vpCameraParameters cam(389.117, 390.358, 342.182, 272.752);
  tracker.setCameraParameters(cam);
  tracker.setDisplayMovingEdges(true);

  // Wait for the image to be initialized.
  ros::Rate loop_rate(10);
  while (!I.getWidth() || !I.getHeight())
    {
      ros::spinOnce();
      loop_rate.sleep();
    }

  vpDisplayX d(I, I.getWidth(), I.getHeight(),
	       "ViSP MBT tracker initialization");

  bool ok = false;
  vpHomogeneousMatrix cMo;
  vpImagePoint point (10, 10);
  while (!ok)
    {
      try
	{
	  // Initialize.
	  vpDisplay::display(I);
	  vpDisplay::flush(I);
	  tracker.initClick(I, init_path.c_str());
	  tracker.getPose(cMo);

	  // Track once to make sure initialization is correct.
	  vpDisplay::display(I);
	  tracker.track(I);
	  tracker.display(I, cMo, cam, vpColor::red);
	  vpDisplay::displayCharString
	    (I, point, "first tracking", vpColor::red);
	  vpDisplay::flush(I);
	  vpDisplay::getClick(I);

	  tracker.getPose(cMo);
	  ok = true;
	}
      catch(...)
	{
	  ROS_ERROR("failed to initialize. Retrying...");
	}
    }

  ROS_INFO_STREAM("Initialization done, sending initial cMo:\n" << cMo);

  ros::ServiceClient client = n.serviceClient<visp_tracker::Init>(init_service);
  visp_tracker::Init srv;

  srv.request.model_path.data = model_path;
  srv.request.model_name.data = model_name;
  srv.request.model_configuration.data = model_configuration;
  vpHomogeneousMatrixToTransform(srv.request.initial_cMo, cMo);

  if (client.call(srv))
  {
    if (srv.response.initialization_succeed)
      ROS_INFO("Tracker initialized with success.");
    else
      {
	ROS_ERROR("Failed to initialize tracker.");
	return 2;
      }
  }
  else
  {
    ROS_ERROR("Failed to call service init");
    return 1;
  }

  return 0;
}
