#include <cstdlib>
#include <fstream>

#include <boost/filesystem.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <ros/param.h>
#include <visp_tracker/Init.h>

#include <visp/vpMe.h>

#define protected public
#include <visp/vpMbEdgeTracker.h>
#undef protected

#include <visp/vpDisplayX.h>

#include "conversion.hh"
#include "callbacks.hh"
#include "file.hh"

typedef vpImage<unsigned char> image_t;

int main(int argc, char **argv)
{
  std::string image_topic;
  std::string model_path;
  std::string model_name;
  std::string model_configuration;
  std::string init_service;
  vpMe moving_edge;

  image_t I;

  vpMbEdgeTracker tracker;
  tracker.resetTracker();

  // Initialization.
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

  ros::param::param("~vpme_mask_size", moving_edge.mask_size, 7);
  ros::param::param("~vpme_n_mask", moving_edge.n_mask, 180);
  ros::param::param("~vpme_range", moving_edge.range, 8);
  ros::param::param("~vpme_threshold", moving_edge.threshold, 100.);
  ros::param::param("~vpme_mu1", moving_edge.mu1, 0.5);
  ros::param::param("~vpme_mu2", moving_edge.mu2, 0.5);
  ros::param::param("~vpme_sample_step", moving_edge.sample_step, 1.);
  ros::param::param("~vpme_ntotal_sample", moving_edge.ntotal_sample, 1000);

  // Camera subscriber.
  image_transport::CameraSubscriber sub =
    it.subscribeCamera(image_topic, 100, bindImageCallback(I));

  // Model loading.
  boost::filesystem::path vrml_path =
    getModelFileFromModelName(model_name, model_path);
  boost::filesystem::path init_path =
    getInitFileFromModelName(model_name, model_path);

  ROS_INFO_STREAM("VRML file: " << vrml_path);
  ROS_INFO_STREAM("Init file: " << init_path);

  // Check that required files exist.
  if (!boost::filesystem::is_regular_file(vrml_path))
    {
      ROS_ERROR_STREAM("VRML model " << vrml_path << " is not a regular file.");
      return 1;
    }
  if (!boost::filesystem::is_regular_file(init_path))
    {
      ROS_ERROR_STREAM("Init file " << vrml_path << " is not a regular file.");
      return 1;
    }

  // Load the 3d model.
  try
    {
      ROS_DEBUG_STREAM("Trying to load the model "
		<< vrml_path.external_file_string());
      tracker.loadModel(vrml_path.external_file_string().c_str());
      ROS_INFO("Model has been successfully loaded.");

      ROS_INFO_STREAM("Nb hidden faces: "
		      << tracker.faces.getPolygon().nbElements());
      ROS_INFO_STREAM("Nb line: " << tracker.Lline.nbElements());
      ROS_INFO_STREAM("nline: " << tracker.nline);
      ROS_INFO_STREAM("Visible faces: " << tracker.nbvisiblepolygone);
    }
  catch(...)
    {
      ROS_ERROR_STREAM("Failed to load the model " << vrml_path);
      return 1;
    }

  // Tracker initialization.

  // - Camera
  //FIXME: replace by real camera parameters of the rectified camera.
  vpCameraParameters cam(389.117, 390.358, 342.182, 272.752);
  tracker.setCameraParameters(cam);
  tracker.setDisplayMovingEdges(true);

  // - Moving edges.
  tracker.setMovingEdge(moving_edge);

  // Display camera parameters and moving edges settings.
  ROS_INFO_STREAM(cam);
  moving_edge.print();

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
	  std::string init_file
	    (init_path.replace_extension().external_file_string());
	  tracker.initClick(I, init_file.c_str());
	  tracker.getPose(cMo);

	  ROS_INFO_STREAM("initial pose [tx,ty,tz,tux,tuy,tuz]:\n"
			  << vpPoseVector(cMo));

	  // Track once to make sure initialization is correct.
	  vpDisplay::display(I);
	  tracker.track(I);
	  tracker.display(I, cMo, cam, vpColor::red, 2);
	  vpDisplay::displayCharString
	    (I, point, "first tracking", vpColor::red);
	  vpDisplay::flush(I);
	  vpDisplay::getClick(I);

	  tracker.getPose(cMo);
	  ok = true;
	}
      catch(const std::string& str)
	{
	  ROS_ERROR_STREAM("failed to initialize: " << str << ", retrying...");
	}
      catch(...)
	{
	  ROS_ERROR("failed to initialize, retrying...");
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
