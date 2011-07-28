#include <cstdlib>
#include <fstream>
#include <sstream>

#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <boost/optional.hpp>

#include <ros/ros.h>
#include <ros/param.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>

#include <visp/vpMbEdgeTracker.h>
#include <visp/vpDisplayX.h>

#include <visp_tracker/MovingEdgeSites.h>
#include <visp_tracker/TrackingResult.h>

#include "conversion.hh"
#include "callbacks.hh"
#include "file.hh"

typedef vpImage<unsigned char> image_t;

typedef message_filters::sync_policies::ApproximateTime<
  sensor_msgs::Image, visp_tracker::TrackingResult, visp_tracker::MovingEdgeSites
  > syncPolicy_t;


void callback(image_t& image,
	      boost::optional<vpHomogeneousMatrix>& cMo,
	      visp_tracker::MovingEdgeSites::ConstPtr& sites,

	      const sensor_msgs::ImageConstPtr& imageConst,
	      const visp_tracker::TrackingResult::ConstPtr& trackingResult,
	      const visp_tracker::MovingEdgeSites::ConstPtr& sitesConst)
{
  // Copy image.
  try
    {
      rosImageToVisp(image, imageConst);
    }
  catch(std::exception& e)
    {
      ROS_ERROR_STREAM("dropping frame: " << e.what());
    }

  // Copy moving edges sites.
  sites = sitesConst;

  // Copy cMo.
  if (trackingResult->is_tracking)
    {
      cMo = vpHomogeneousMatrix();
      transformToVpHomogeneousMatrix(*cMo, trackingResult->cMo);
    }
  else
    cMo = boost::none;
}

void
displayMovingEdgeSites(image_t& I,
		       visp_tracker::MovingEdgeSites::ConstPtr& sites)
{
  if (!sites)
    return;
  for (unsigned i = 0; i < sites->moving_edge_sites.size(); ++i)
    {
      double x = sites->moving_edge_sites[i].x;
      double y = sites->moving_edge_sites[i].y;
      int suppress = sites->moving_edge_sites[i].suppress;
      vpColor color = vpColor::black;

      switch(suppress)
	{
	case 0:
	  color = vpColor::green;
	  break;
	case 1:
	  color = vpColor::blue;
	  break;
	case 2:
	  color = vpColor::purple;
	  break;
	case 4:
	  color = vpColor::red;
	  break;
	default:
	  ROS_ERROR_THROTTLE(10, "bad suppress value");
	}

      vpDisplay::displayCross(I, vpImagePoint(x, y), 3, color, 1);
    }
}

int main(int argc, char **argv)
{
  std::string image_topic;

  std::string model_path;
  std::string model_name;
  std::string model_configuration;

  std::string camera_parameters_service;

  std::string tracker_result;
  std::string moving_edge_sites;

  image_t I;

  bool displayMovingEdges;

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
  ros::param::param<std::string>
    ("~moving_edge_sites",
     moving_edge_sites, "/tracker_mbt/moving_edge_sites");

  ros::param::param<bool>("~display_moving_edges", displayMovingEdges, true);

  // Tracker initialization.
  vpMbEdgeTracker tracker;

  // Model loading.
  boost::filesystem::path vrml_path =
    getModelFileFromModelName(model_name, model_path);

  ROS_INFO_STREAM("VRML file: " << vrml_path);

  try
    {
      ROS_DEBUG_STREAM("Trying to load the model " << vrml_path);
      tracker.loadModel(vrml_path.external_file_string().c_str());
    }
  catch(...)
    {
      ROS_ERROR_STREAM("Failed to load the model " << vrml_path);
      return 1;
    }
  ROS_INFO("Model has been successfully loaded.");

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

  // Subscribe to camera and tracker synchronously.
  boost::optional<vpHomogeneousMatrix> cMo;
  visp_tracker::MovingEdgeSites::ConstPtr sites;

  image_transport::SubscriberFilter imageSub
    (it, image_topic, 100);
  message_filters::Subscriber<visp_tracker::TrackingResult> trackingResultSub
    (n, tracker_result, 100);
  message_filters::Subscriber<visp_tracker::MovingEdgeSites>
    movingEdgeSitesSub(n, moving_edge_sites, 100);

  message_filters::Synchronizer<syncPolicy_t> sync
    (syncPolicy_t(100), imageSub, trackingResultSub, movingEdgeSitesSub);

  sync.registerCallback
    (boost::bind(callback,
		 boost::ref(I), boost::ref(cMo), boost::ref(sites),
		 _1, _2, _3));
    

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
      if (displayMovingEdges)
	displayMovingEdgeSites(I, sites);
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
