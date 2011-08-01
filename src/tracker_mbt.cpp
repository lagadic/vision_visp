#include <stdexcept>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <ros/param.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>

#include <visp_tracker/Init.h>
#include <visp_tracker/MovingEdgeSites.h>
#include <visp_tracker/TrackingResult.h>
#include <visp_tracker/TrackingMetaData.h>

#include <boost/bind.hpp>
#include <visp/vpImage.h>
#include <visp/vpImageConvert.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpMbEdgeTracker.h>

#include "conversion.hh"
#include "callbacks.hh"
#include "file.hh"
#include "names.hh"

// TODO:
// - add a topic allowing to suggest an estimation of the cMo
// - handle automatic reset when tracking is lost.

typedef vpImage<unsigned char> image_t;

typedef boost::function<bool (visp_tracker::Init::Request&,
			      visp_tracker::Init::Response& res)>
init_callback_t;

typedef boost::function<bool (visp_tracker::TrackingMetaData::Request&,
			      visp_tracker::TrackingMetaData::Response& res)>
trackingMetaData_callback_t;

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
		  unsigned long& lastTrackedImage,
		  visp_tracker::Init::Request& req,
		  visp_tracker::Init::Response& res)
{
  ROS_INFO("Initialization request received.");

  // Update the parameters.
  ros::param::set("model_path", req.model_path.data);
  ros::param::set("model_name", req.model_name.data);

  ros::param::set("vpme_mask_size", (int)req.moving_edge.mask_size);
  ros::param::set("vpme_n_mask", (int)req.moving_edge.n_mask);
  ros::param::set("vpme_range", (int)req.moving_edge.range);
  ros::param::set("vpme_threshold", req.moving_edge.threshold);
  ros::param::set("vpme_mu1", req.moving_edge.mu1);
  ros::param::set("vpme_mu2", req.moving_edge.mu2);
  ros::param::set("vpme_sample_step", (int)req.moving_edge.sample_step);
  ros::param::set("vpme_ntotal_sample", (int)req.moving_edge.ntotal_sample);

  model_path = req.model_path.data;
  model_name = req.model_name.data;

  // Load moving edges.
  vpMe moving_edge;
  convertInitRequestToVpMe(req, tracker, moving_edge);

  //FIXME: not sure if this is needed.
  moving_edge.initMask();

  // Reset the tracker and the node state.
  tracker.resetTracker();
  state = WAITING_FOR_INITIALIZATION;
  lastTrackedImage = 0;

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

bool trackingMetaDataCallback(const std::string& camera_prefix,
			      const std::string& model_path,
			      const std::string& model_name,
			      visp_tracker::TrackingMetaData::Request&,
			      visp_tracker::TrackingMetaData::Response& res)
{
  res.camera_prefix.data = camera_prefix;
  res.model_path.data = model_path;
  res.model_name.data = model_name;
  return true;
}


init_callback_t bindInitCallback(State& state,
				 vpMbEdgeTracker& tracker,
				 image_t& image,
				 std::string& model_path,
				 std::string& model_name,
				 unsigned long& lastTrackedImage)
{
  return boost::bind(initCallback,
		     boost::ref(state),
		     boost::ref(tracker),
		     boost::ref(image),
		     boost::ref(model_path),
		     boost::ref(model_name),
		     boost::ref(lastTrackedImage),
		     _1, _2);
}

trackingMetaData_callback_t
bindTrackingMetaDataCallback(const std::string& camera_prefix,
			     const std::string& model_path,
			     const std::string& model_name)
{
  return boost::bind(trackingMetaDataCallback,
		     boost::ref(camera_prefix),
		     boost::ref(model_path),
		     boost::ref(model_name),
		     _1, _2);
}


void
updateMovingEdgeSites(visp_tracker::MovingEdgeSites& sites,
		      vpMbEdgeTracker& tracker)
{
  sites.moving_edge_sites.clear();

  tracker.getLline()->front();
  while (!tracker.getLline()->outside())
    {
      vpMbtDistanceLine* l = tracker.getLline()->value();
      if (l && l->isVisible() && l->meline)
	{
	  l->meline->list.front();
	  while (!l->meline->list.outside())
	    {
	      vpMeSite pix = l->meline->list.value();
	      visp_tracker::MovingEdgeSite movingEdgeSite;
	      movingEdgeSite.x = pix.ifloat;
	      movingEdgeSite.y = pix.jfloat;
	      movingEdgeSite.suppress = pix.suppress;
	      sites.moving_edge_sites.push_back(movingEdgeSite);
	      l->meline->list.next();
	    }
	}
      tracker.getLline()->next();
    }
}


int main(int argc, char **argv)
{
  State state = WAITING_FOR_INITIALIZATION;
  std::string camera_prefix;
  std::string model_path("");
  std::string model_name("");
  vpMe moving_edge;

  image_t I;

  ros::init(argc, argv, "tracker_mbt");

  ros::NodeHandle n("tracker_mbt");
  image_transport::ImageTransport it(n);

  // Parameters.
  ros::param::param<std::string>("~camera_prefix", camera_prefix, "");

  // Compute topic and services names.
  const std::string rectified_image_topic =
    ros::names::clean(camera_prefix + "/image_rect");

  visp_tracker::MovingEdgeConfig defaultMovingEdge =
    visp_tracker::MovingEdgeConfig::__getDefault__();
  ros::param::param("vpme_mask_size", moving_edge.mask_size,
		    defaultMovingEdge.mask_size);
  ros::param::param("vpme_n_mask", moving_edge.n_mask,
		    defaultMovingEdge.n_mask);
  ros::param::param("vpme_range", moving_edge.range,
		    defaultMovingEdge.range);
  ros::param::param("vpme_threshold", moving_edge.threshold,
		    defaultMovingEdge.threshold);
  ros::param::param("vpme_mu1", moving_edge.mu1,
		    defaultMovingEdge.mu1);
  ros::param::param("vpme_mu2", moving_edge.mu2,
		    defaultMovingEdge.mu2);
  ros::param::param("vpme_sample_step", moving_edge.sample_step,
		    defaultMovingEdge.sample_step);
  ros::param::param("vpme_ntotal_sample", moving_edge.ntotal_sample,
		    defaultMovingEdge.ntotal_sample);

  // Result publisher.
  ros::Publisher result_pub =
    n.advertise<visp_tracker::TrackingResult>
    (visp_tracker::result_topic, 1000);

  // Moving edge sites publisher.
  ros::Publisher moving_edge_sites_pub =
    n.advertise<visp_tracker::MovingEdgeSites>
    (visp_tracker::moving_edge_sites_topic, 1000);

  // Camera subscriber.
  std_msgs::Header header;
  sensor_msgs::CameraInfoConstPtr info;
  image_transport::CameraSubscriber sub =
    it.subscribeCamera(rectified_image_topic, 100,
		       bindImageCallback(I, header, info));

  // Initialization.
  ros::Rate loop_rate(10);

  vpMbEdgeTracker tracker;
  moving_edge.initMask();
  tracker.setMovingEdge(moving_edge);

  // Dynamic reconfigure.
  dynamic_reconfigure::Server<visp_tracker::MovingEdgeConfig> reconfigureSrv;
  dynamic_reconfigure::Server<visp_tracker::MovingEdgeConfig>::CallbackType f;
  f = boost::bind(&reconfigureCallback, boost::ref(tracker),
		  boost::ref(moving_edge), _1, _2);
  reconfigureSrv.setCallback(f);

  // Wait for the image to be initialized.
  while (!I.getWidth() || !I.getHeight())
    {
      ros::spinOnce();
      loop_rate.sleep();
    }

  // Tracker initialization.
  vpCameraParameters cam;
  try
    {
      initializeVpCameraFromCameraInfo(cam, info);
    }
  catch(const std::exception& e)
    {
      ROS_ERROR_STREAM("failed to initialize camera: " << e.what());
      return 1;
    }
  tracker.setCameraParameters(cam);
  tracker.setDisplayMovingEdges(false);

  ROS_INFO_STREAM(cam);

  // Service declaration.
  unsigned long lastTrackedImage = 0;

  ros::ServiceServer initService = n.advertiseService
    (visp_tracker::init_service,
     bindInitCallback(state, tracker, I,
		      model_path, model_name,
		      lastTrackedImage));

  ros::ServiceServer trackingMetaDataService = n.advertiseService
    (visp_tracker::tracking_meta_data_service,
     bindTrackingMetaDataCallback(boost::ref(camera_prefix),
				  boost::ref(model_path),
				  boost::ref(model_name)));


  // Main loop.
  ros::Rate loop_rate_tracking(200);
  vpHomogeneousMatrix cMo;
  visp_tracker::TrackingResult result;
  visp_tracker::MovingEdgeSites moving_edge_sites;
  unsigned long lastHeaderSeq = 0;

  while (ros::ok())
    {
      // When a camera sequence is played several times,
      // the seq id will decrease, in this case we want to
      // continue the tracking.
      if (header.seq < lastHeaderSeq)
	lastTrackedImage = 0;
      lastHeaderSeq = header.seq;

      if (lastTrackedImage < header.seq)
	{
	  lastTrackedImage = header.seq;
	    cMo.eye();
	    if (state == TRACKING)
	      try
		{

	    tracker.track(I);
	    tracker.getPose(cMo);
		}
	      catch(...)
		{
		  ROS_WARN("Tracking lost.");
		  state = LOST;
		}

	    // Publish the tracking result.
	    result.header = header;
	    result.is_tracking = state == TRACKING;

	    if (state == TRACKING)
	      vpHomogeneousMatrixToTransform(result.cMo, cMo);
	    result_pub.publish(result);

	    if (state == TRACKING)
	      updateMovingEdgeSites(moving_edge_sites, tracker);
	    else
	      moving_edge_sites.moving_edge_sites.clear();
	    moving_edge_sites.header = header;
	    moving_edge_sites_pub.publish(moving_edge_sites);
	}

      ros::spinOnce();
      loop_rate_tracking.sleep();
    }

  return 0;
}
