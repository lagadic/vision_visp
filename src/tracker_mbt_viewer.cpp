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
#include <visp_tracker/TrackingMetaData.h>

#include "conversion.hh"
#include "callbacks.hh"
#include "file.hh"
#include "names.hh"

typedef vpImage<unsigned char> image_t;

typedef message_filters::sync_policies::ApproximateTime<
  sensor_msgs::Image, sensor_msgs::CameraInfo,
  visp_tracker::TrackingResult, visp_tracker::MovingEdgeSites
  > syncPolicy_t;

namespace
{
  void increment(unsigned int* value)
  {
    ++(*value);
  }
} // end of anonymous namespace.

void callback(image_t& image,
	      sensor_msgs::CameraInfoConstPtr& info,
	      boost::optional<vpHomogeneousMatrix>& cMo,
	      visp_tracker::MovingEdgeSites::ConstPtr& sites,

	      const sensor_msgs::ImageConstPtr& imageConst,
	      const sensor_msgs::CameraInfoConstPtr& infoConst,
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

  // Copy moving camera infos and edges sites.
  info = infoConst;
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
checkInputsSynchronized(unsigned& allReceived,
			unsigned& imageReceived,
			unsigned& cameraInfoReceived,
			unsigned& resultReceived,
			unsigned& movingEdgeSitesReceived)
{
  const unsigned threshold = 3 * allReceived;

  if (imageReceived > threshold
      || cameraInfoReceived > threshold
      || resultReceived > threshold
      || movingEdgeSitesReceived > threshold)
    {
      ROS_WARN
	("[visp_tracker] Low number of synchronized tuples"
	 "image/camera info/result/moving edge received.\n"
	 "Images received: %d\n"
	 "Camera info received: %d\n"
	 "Results received: %d\n"
	 "Moving edges received: %d\n"
	 "Synchronized triplets: %d\n"
	 "Possible issues:\n"
	 "\t* The network is too slow. One or more images are dropped from each"
	 "triplet.",
	 imageReceived, cameraInfoReceived, resultReceived,
	 movingEdgeSitesReceived, allReceived);
    }
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
  std::string tracker_prefix;

  image_t I;

  ros::init(argc, argv, "tracker_mbt_viewer");

  ros::NodeHandle n;
  image_transport::ImageTransport it(n);

  // Parameters.
  ros::param::param<std::string>("~tracker_prefix",
				 tracker_prefix,
				 visp_tracker::default_tracker_prefix);

  // Get meta-data.
  //FIXME: we suppose here that these info cannot change at runtime.
  const std::string tracking_meta_data_service =
    ros::names::clean
    (tracker_prefix + "/" + visp_tracker::tracking_meta_data_service);

  ros::ServiceClient client = n.serviceClient<visp_tracker::TrackingMetaData>
    (tracking_meta_data_service);
  visp_tracker::TrackingMetaData srv;
  if (!client.call(srv))
    {
      ROS_ERROR_STREAM("failed to retrieve tracking meta-data.");
      return 1;
    }

  // Compute topic and services names.
  const std::string rectified_image_topic =
    ros::names::clean(srv.response.camera_prefix.data + "/image_rect");
  const std::string camera_info_topic =
    ros::names::clean(srv.response.camera_prefix.data + "/camera_info");

  const std::string result_topic =
    ros::names::clean(tracker_prefix + "/" + visp_tracker::result_topic);
  const std::string moving_edge_sites_topic =
    ros::names::clean
    (tracker_prefix + "/" + visp_tracker::moving_edge_sites_topic);

  // Tracker initialization.
  vpMbEdgeTracker tracker;

  // Model loading.
  boost::filesystem::path vrml_path =
    getModelFileFromModelName(srv.response.model_name.data,
			      srv.response.model_path.data);

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

  // Subscribe to camera and tracker synchronously.
  sensor_msgs::CameraInfoConstPtr info;
  boost::optional<vpHomogeneousMatrix> cMo;
  visp_tracker::MovingEdgeSites::ConstPtr sites;

  image_transport::SubscriberFilter imageSub
    (it, rectified_image_topic, 100);
  message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoSub
    (n, camera_info_topic, 100);
  message_filters::Subscriber<visp_tracker::TrackingResult> trackingResultSub
    (n, result_topic, 100);
  message_filters::Subscriber<visp_tracker::MovingEdgeSites>
    movingEdgeSitesSub(n, moving_edge_sites_topic, 100);

  message_filters::Synchronizer<syncPolicy_t> sync
    (syncPolicy_t(100),
     imageSub, cameraInfoSub,
     trackingResultSub, movingEdgeSitesSub);

  sync.registerCallback
    (boost::bind(callback,
		 boost::ref(I), boost::ref(info),
		 boost::ref(cMo), boost::ref(sites),
		 _1, _2, _3, _4));

  // Trigger a warning if no synchronized triplets are received during 30s.
  unsigned allReceived = 0;
  unsigned imageReceived = 0;
  unsigned cameraInfoReceived = 0;
  unsigned resultReceived = 0;
  unsigned movingEdgeSitesReceived = 0;

  imageSub.registerCallback(boost::bind(increment, &imageReceived));
  cameraInfoSub.registerCallback(boost::bind(increment, &cameraInfoReceived));
  trackingResultSub.registerCallback(boost::bind(increment, &resultReceived));
  movingEdgeSitesSub.registerCallback
    (boost::bind(increment, &movingEdgeSitesReceived));

  ros::WallTimer checkSyncedTimer =
    n.createWallTimer
    (ros::WallDuration(30.0),
     boost::bind(checkInputsSynchronized,
		 allReceived,
		 imageReceived,
		 cameraInfoReceived,
		 resultReceived,
		 movingEdgeSitesReceived));

  // Wait for the image to be initialized.
  ros::Rate loop_rate(10);
  while (!I.getWidth() || !I.getHeight())
    {
      ros::spinOnce();
      loop_rate.sleep();
      if (!ros::ok())
	exit(1);
    }


  // Camera.
  vpCameraParameters cam;
  try
    {
      initializeVpCameraFromCameraInfo(cam, info);
    }
  catch(std::exception& e)
    {
      ROS_ERROR_STREAM("failed to initialize camera: " << e.what());
      return 1;
    }
  tracker.setCameraParameters(cam);
  tracker.setDisplayMovingEdges(true);

  vpDisplayX d(I, I.getWidth(), I.getHeight(), "ViSP MBT tracker (viewer)");
  vpImagePoint point (10, 10);
  while (ros::ok())
    {
      vpDisplay::display(I);
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
      if (!ros::ok())
	exit(1);
    }
}
