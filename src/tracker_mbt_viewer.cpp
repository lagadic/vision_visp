#include <cstdlib>
#include <fstream>
#include <sstream>

#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <boost/optional.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <ros/param.h>

#include <visp/vpMbEdgeTracker.h>
#include <visp/vpDisplayX.h>

#include <visp_tracker/TrackingResult.h>

#include "conversion.hh"
#include "callbacks.hh"
#include "file.hh"

typedef vpImage<unsigned char> image_t;

typedef boost::function<void (const visp_tracker::TrackingResult::ConstPtr&)>
result_callback_t;


void resultCallback(boost::optional<vpHomogeneousMatrix>& cMo,
		    const visp_tracker::TrackingResult::ConstPtr& msg)
{
  if (!msg->is_tracking)
    {
      cMo = boost::none;
      return;
    }

  cMo = vpHomogeneousMatrix();
  transformToVpHomogeneousMatrix(*cMo, msg->cMo.transform);
}

result_callback_t bindResultCallback (boost::optional<vpHomogeneousMatrix>& cMo)
{
  return boost::bind(resultCallback, boost::ref(cMo), _1);
}

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

  std::string camera_parameters_service;

  std::string tracker_result;

  image_t I;

  ros::init(argc, argv, "tracker_mbt_viewer");

  ros::NodeHandle n;
  image_transport::ImageTransport it(n);

  // Parameters.
  ros::param::param<std::string>("~image", image_topic, "/camera/image_raw");

  ros::param::param<std::string>("~model_path", model_path, "");
  ros::param::param<std::string>("~model_name", model_name, "");
  ros::param::param<std::string>("~model_configuration",
				 model_configuration, "default");

  ros::param::param<std::string>
    ("~camera_parameters_service",
     camera_parameters_service, "/tracker_mbt/camera_parameters");

  ros::param::param<std::string>("~tracker_result",
				 tracker_result, "/tracker_mbt/result");


  // Camera subscriber.
  image_transport::CameraSubscriber sub =
    it.subscribeCamera(image_topic, 100, bindImageCallback(I));

  // Tracker initialization.
  vpMbEdgeTracker tracker;

  // Model loading.
  std::string vrml_path =
    getModelFileFromModelName(model_name, model_path).external_file_string();

  ROS_DEBUG("VRML file: %s", vrml_path.c_str());

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

  // Camera.
  boost::optional<vpCameraParameters> cameraParametersOpt =
    loadCameraParameters(n, camera_parameters_service);
  if (!cameraParametersOpt)
    {
      ROS_ERROR_STREAM("failed to call service camera_parameters");
      return 1;
    }
  vpCameraParameters cam(*cameraParametersOpt);
  tracker.setCameraParameters(cam);
  tracker.setDisplayMovingEdges(true);

  // Subscribe to tracker.
  boost::optional<vpHomogeneousMatrix> cMo;
  ros::Subscriber subResult =
    n.subscribe(tracker_result, 1000, bindResultCallback(cMo));

  // Wait for the image to be initialized.
  ros::Rate loop_rate(10);
  while (!I.getWidth() || !I.getHeight())
    {
      ros::spinOnce();
      loop_rate.sleep();
    }

  vpDisplayX d(I, I.getWidth(), I.getHeight(), "ViSP MBT tracker (viewer)");
  vpImagePoint point (10, 10);
  while (ros::ok())
    {
      vpDisplay::display(I);
      if (cMo)
	{
	  tracker.setPose(*cMo);
	  tracker.display(I, *cMo, cam, vpColor::red);

	  ROS_DEBUG_STREAM_THROTTLE(10, "cMo:\n" << *cMo);

	  boost::format fmt("tracking (x=%f y=%f z=%f)");
	  fmt % (*cMo)[0][3] % (*cMo)[1][3] % (*cMo)[2][3];
	  vpDisplay::displayCharString
	    (I, point, fmt.str().c_str(), vpColor::red);
	}
      else
	vpDisplay::displayCharString(I, point, "tracking failed", vpColor::red);
      vpDisplay::flush(I);

      ros::spinOnce();
      loop_rate.sleep();
    }
}
