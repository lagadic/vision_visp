#include <cstdlib>
#include <fstream>

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/format.hpp>
#include <boost/optional.hpp>

#include <ros/ros.h>
#include <ros/param.h>
#include <dynamic_reconfigure/server.h>
#include <image_proc/advertisement_checker.h>
#include <image_transport/image_transport.h>
#include <visp_tracker/Init.h>
#include <visp_tracker/MovingEdgeConfig.h>

#include <visp/vpMe.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpPose.h>

#define protected public
#include <visp/vpMbEdgeTracker.h>
#undef protected

#include <visp/vpDisplayX.h>

#include "conversion.hh"
#include "callbacks.hh"
#include "file.hh"
#include "names.hh"

#include "tracker-client.hh"


namespace visp_tracker
{
  TrackerClient::TrackerClient(unsigned queueSize)
    : queueSize_(queueSize),
      nodeHandle_(),
      imageTransport_(nodeHandle_),
      image_(),
      modelPath_(),
      modelName_(),
      cameraPrefix_(),
      trackerPrefix_(),
      rectifiedImageTopic_(),
      cameraInfoTopic_(),
      initService_(),
      vrmlPath_(),
      initPath_(),
      cameraSubscriber_(),
      reconfigureSrv_(),
      movingEdge_(),
      cameraParameters_(),
      tracker_(),
      checkInputs_()
  {
    tracker_.resetTracker();

    // Parameters.
    ros::param::param<std::string>("~camera_prefix", cameraPrefix_, "");
    ros::param::param<std::string>("~tracker_prefix",
				   trackerPrefix_,
				   visp_tracker::default_tracker_prefix);

    ros::param::param<std::string>("~model_path", modelPath_,
				   visp_tracker::default_model_path);
    ros::param::param<std::string>("~model_name", modelName_, "");

    if (modelName_ == "")
      throw std::runtime_error("empty model");

    // Compute topic and services names.
    rectifiedImageTopic_ =
      ros::names::clean(cameraPrefix_ + "/image_rect");
    cameraInfoTopic_ =
      ros::names::clean(cameraPrefix_ + "/camera_info");

    initService_ =
    ros::names::clean(trackerPrefix_ + "/" + visp_tracker::init_service);

    // Check for subscribed topics.
    checkInputs();

    // Set callback for dynamic reconfigure.
    reconfigureCallback_t f =
      boost::bind(&reconfigureCallback, boost::ref(tracker_),
		  boost::ref(image_), boost::ref(movingEdge_), _1, _2);
    reconfigureSrv_.setCallback(f);

    // Camera subscriber.
    cameraSubscriber_ = imageTransport_.subscribeCamera
      (rectifiedImageTopic_, queueSize_,
       bindImageCallback(image_, header_, info_));

    // Model loading.
    vrmlPath_ = getModelFileFromModelName(modelName_, modelPath_);
    initPath_ = getInitFileFromModelName(modelName_, modelPath_);

    ROS_INFO_STREAM("VRML file: " << vrmlPath_);
    ROS_INFO_STREAM("Init file: " << initPath_);

    // Check that required files exist.
    if (!boost::filesystem::is_regular_file(vrmlPath_))
      {
	boost::format fmt("VRML model %1% is not a regular file");
	fmt % vrmlPath_;
	throw std::runtime_error(fmt.str());
      }
    if (!boost::filesystem::is_regular_file(initPath_))
      {
	boost::format fmt("Initialization file %1% is not a regular file");
	fmt % initPath_;
	throw std::runtime_error(fmt.str());
      }

    // Load the 3d model.
    loadModel();

    // Wait for the image to be initialized.
    waitForImage();
    if (!ros::ok())
      return;
    if (!image_.getWidth() || !image_.getHeight())
      throw std::runtime_error("failed to retrieve image");

    // Tracker initialization.
    // - Camera
    initializeVpCameraFromCameraInfo(cameraParameters_, info_);
    tracker_.setCameraParameters(cameraParameters_);
    tracker_.setDisplayMovingEdges(true);

    // - Moving edges.
    movingEdge_.initMask();
    tracker_.setMovingEdge(movingEdge_);

    // Display camera parameters and moving edges settings.
    ROS_INFO_STREAM(cameraParameters_);
    movingEdge_.print();
  }

  void
  TrackerClient::checkInputs()
  {
    ros::V_string topics;
    topics.push_back(rectifiedImageTopic_);
    topics.push_back(cameraInfoTopic_);
    checkInputs_.start(topics, 60.0);
  }

  void
  TrackerClient::spin()
  {
    vpDisplayX d(image_, image_.getWidth(), image_.getHeight(),
		 "ViSP MBT tracker initialization");

    ros::Rate loop_rate_tracking(200);
    bool ok = false;
    vpHomogeneousMatrix cMo;
    vpImagePoint point (10, 10);
    while (!ok && ros::ok())
      {
	try
	  {
	    // Initialize.
	    vpDisplay::display(image_);
	    vpDisplay::flush(image_);
	    initClick();
	    tracker_.getPose(cMo);

	    ROS_INFO_STREAM("initial pose [tx,ty,tz,tux,tuy,tuz]:\n"
			    << vpPoseVector(cMo));

	    // Track once to make sure initialization is correct.
	    vpImagePoint ip;
	    vpMouseButton::vpMouseButtonType button = vpMouseButton::button1;
	    do
	      {
		vpDisplay::display(image_);
		tracker_.track(image_);
		tracker_.display(image_, cMo, cameraParameters_,
				 vpColor::red, 2);
		vpDisplay::displayCharString
		  (image_, point, "tracking, click to initialize tracker",
		   vpColor::red);
		vpDisplay::flush(image_);
		tracker_.getPose(cMo);

		ros::spinOnce();
		loop_rate_tracking.sleep();
		if (!ros::ok())
		  return;
	      }
	    while(!vpDisplay::getClick(image_, ip, button, false));
	    ok = true;
	  }
	catch(const std::string& str)
	  {
	    ROS_ERROR_STREAM("failed to initialize: "
			     << str << ", retrying...");
	  }
	catch(...)
	  {
	    ROS_ERROR("failed to initialize, retrying...");
	  }
      }

    ROS_INFO_STREAM("Initialization done, sending initial cMo:\n" << cMo);
    sendcMo(cMo);
  }

  void
  TrackerClient::sendcMo(const vpHomogeneousMatrix& cMo)
  {
    ros::ServiceClient client =
      nodeHandle_.serviceClient<visp_tracker::Init>(initService_);
    visp_tracker::Init srv;

    srv.request.model_path.data = modelPath_;
    srv.request.model_name.data = modelName_;
    vpHomogeneousMatrixToTransform(srv.request.initial_cMo, cMo);

    convertVpMeToInitRequest(movingEdge_, tracker_, srv);

    if (client.call(srv))
      {
	if (srv.response.initialization_succeed)
	  ROS_INFO("Tracker initialized with success.");
	else
	  throw std::runtime_error("failed to initialize tracker.");
      }
    else
      throw std::runtime_error("failed to call service init");
  }

  void
  TrackerClient::loadModel()
  {
    try
      {
	ROS_DEBUG_STREAM("Trying to load the model "
			 << vrmlPath_.external_file_string());
	tracker_.loadModel(vrmlPath_.external_file_string().c_str());
	ROS_INFO("VRML model has been successfully loaded.");

	ROS_DEBUG_STREAM("Nb hidden faces: "
			<< tracker_.faces.getPolygon().nbElements());
	ROS_DEBUG_STREAM("Nb line: " << tracker_.Lline.nbElements());
	ROS_DEBUG_STREAM("nline: " << tracker_.nline);
	ROS_DEBUG_STREAM("Visible faces: " << tracker_.nbvisiblepolygone);
      }
    catch(...)
      {
	boost::format fmt("Failed to load the model %1%");
	fmt % vrmlPath_;
	throw std::runtime_error(fmt.str());
      }
  }

  vpHomogeneousMatrix
  TrackerClient::loadInitialPose()
  {
    vpHomogeneousMatrix cMo;
    cMo.eye();

    boost::filesystem::path initialPose =
      getInitialPoseFileFromModelName(modelName_, modelPath_);
    boost::filesystem::ifstream file(initialPose);
    if (!file.good())
      {
	ROS_WARN_STREAM("failed to load initial pose: " << initialPose << "\n"
			<< "using identity as initial pose");
	return cMo;
      }

    vpPoseVector pose;
    for (unsigned i = 0; i < 6; ++i)
      if (file.good())
	file >> pose[i];
      else
	{
	  ROS_WARN("failed to parse initial pose file");
	  return cMo;
	}
    cMo.buildFrom(pose);
    return cMo;
  }

  void
  TrackerClient::saveInitialPose(const vpHomogeneousMatrix& cMo)
  {
    boost::filesystem::path initialPose =
      getInitialPoseFileFromModelName(modelName_, modelPath_);
    boost::filesystem::ofstream file(initialPose);
    if (!file.good())
      {
	ROS_WARN_STREAM("failed to save initial pose: " << initialPose);
	return;
      }
    vpPoseVector pose;
    pose.buildFrom(cMo);
    file << pose;
  }

  TrackerClient::points_t
  TrackerClient::loadInitializationPoints()
  {
    points_t points;

    boost::filesystem::path init =
      getInitFileFromModelName(modelName_, modelPath_);
    boost::filesystem::ifstream file(init);
    if (!file.good())
      {
	boost::format fmt("failed to load initialization points: %1");
	fmt % init;
	throw std::runtime_error(fmt.str());
      }

    unsigned npoints = 0;
    file >> npoints;
    if (!file.good())
      throw std::runtime_error("failed to read initialization file");

    double X = 0., Y = 0., Z = 0.;
    vpPoint point;
    for (unsigned i = 0; i < npoints; ++i)
      {
	if (!file.good())
	  throw std::runtime_error("failed to read initialization file");
	file >> X >> Y >> Z;
	point.setWorldCoordinates(X,Y,Z);
	points.push_back(point);
      }
    return points;
  }

  void
  TrackerClient::initPoint(unsigned& i,
			   points_t& points,
			   imagePoints_t& imagePoints,
			   ros::Rate& rate,
			   vpPose& pose)
  {
    vpImagePoint ip;
    double x = 0., y = 0.;
    boost::format fmt("click on point %u/%u");
    fmt % (i + 1) % points.size(); // list points from 1 to n.

    // Click on the point.
    vpMouseButton::vpMouseButtonType button = vpMouseButton::button1;
    do
      {
	vpDisplay::display(image_);
	vpDisplay::displayCharString
	  (image_, 15, 10,
	   fmt.str().c_str(),
	   vpColor::red);

	for (unsigned j = 0; j < imagePoints.size(); ++j)
	  vpDisplay::displayCross(image_, imagePoints[j], 5, vpColor::green);

	vpDisplay::flush(image_);
	ros::spinOnce();
	rate.sleep();
	if (!ros::ok())
	  return;
      }
    while(!vpDisplay::getClick(image_, ip, button, false));

    imagePoints.push_back(ip);
    vpPixelMeterConversion::convertPoint(cameraParameters_, ip, x, y);
    points[i].set_x(x);
    points[i].set_y(y);
    pose.addPoint(points[i]);
  }

  void
  TrackerClient::initClick()
  {
    ros::Rate loop_rate(200);
    vpHomogeneousMatrix cMo;
    vpImagePoint point (10, 10);

    cMo = loadInitialPose();

    // Show last cMo.
    vpImagePoint ip;
    vpMouseButton::vpMouseButtonType button = vpMouseButton::button1;
    do
      {
	vpDisplay::display(image_);
	tracker_.display(image_, cMo, cameraParameters_, vpColor::green);
	vpDisplay::displayFrame(image_, cMo, cameraParameters_,
				0.05, vpColor::green);
	vpDisplay::displayCharString
	  (image_, 15, 10,
	   "left click to validate, right click to modify initial pose",
	   vpColor::red);
	vpDisplay::flush(image_);
	ros::spinOnce();
	loop_rate.sleep();
	if (!ros::ok())
	  return;
      }
    while(!vpDisplay::getClick(image_, ip, button, false));

    if(button == vpMouseButton::button1)
      {
	tracker_.init(image_, cMo);
	return;
      }

    points_t points = loadInitializationPoints();
    imagePoints_t imagePoints;

    vpPose pose;
    pose.clearPoint();
    bool done = false;
    while (!done)
      {
	// Initialize points.
	for(unsigned i = 0; i < points.size(); ++i)
	  {
	    initPoint(i, points, imagePoints, loop_rate, pose);
	    if (!ros::ok())
	      return;
	  }

	// Compute initial pose.
	vpHomogeneousMatrix cMo1, cMo2;
	pose.computePose(vpPose::LAGRANGE, cMo1);
	double d1 = pose.computeResidual(cMo1);
	pose.computePose(vpPose::DEMENTHON, cMo2);
	double d2 = pose.computeResidual(cMo2);

	if(d1 < d2)
	  cMo = cMo1;
	else
	  cMo = cMo2;
	pose.computePose(vpPose::VIRTUAL_VS, cMo);

	// Confirm result.
	do
	  {
	    vpDisplay::display(image_);
	    tracker_.display(image_, cMo, cameraParameters_, vpColor::green);
	    vpDisplay::displayCharString
	      (image_, 15, 10,
	       "left click to validate, right click to re initialize object",
	       vpColor::red);
	    vpDisplay::flush(image_);
	    ros::spinOnce();
	    loop_rate.sleep();
	    if (!ros::ok())
	      return;
	  }
	while(!vpDisplay::getClick(image_, ip, button, false));

	if(button != vpMouseButton::button1)
	  {
	    pose.clearPoint();
	    imagePoints.clear();
	  }
	else
	  done = true;
      }
    tracker_.init(image_, cMo);
    saveInitialPose(cMo);
  }

  void
  TrackerClient::waitForImage()
  {
    ros::Rate loop_rate(10);
    while (ros::ok()
	   && (!image_.getWidth() || !image_.getHeight()))
      {
	ros::spinOnce();
	loop_rate.sleep();
      }
  }
} // end of namespace visp_tracker.
