#include <cstdlib>
#include <fstream>

#include <boost/bind.hpp>
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
    cMo.reset();

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

  ros::param::param<std::string>("~tracker_result",
				 tracker_result, "/tracker_mbt/result");


  // Camera subscriber.
  image_transport::CameraSubscriber sub =
    it.subscribeCamera(image_topic, 100, bindImageCallback(I));

  // Tracker initialization.
  vpMbEdgeTracker tracker;

  // Model loading.
  std::string vrml_path = getModelFileFromModelName(model_name, model_path);
  std::string init_path = getInitFileFromModelName(model_name, model_path);

  ROS_DEBUG("VRML file: %s", vrml_path.c_str());
  ROS_DEBUG("Init file: %s.init", init_path.c_str());

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


  //FIXME: replace by real camera parameters of the rectified camera.
  vpCameraParameters cam(389.117, 390.358, 342.182, 272.752);
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

	  vpDisplay::displayCharString(I, point, "tracking", vpColor::red);
	}
      else
	vpDisplay::displayCharString(I, point, "tracking failed", vpColor::red);
      vpDisplay::flush(I);

      ros::spinOnce();
      loop_rate.sleep();
    }
}
