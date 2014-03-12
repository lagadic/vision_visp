#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/format.hpp>
#include <boost/optional.hpp>
#include <boost/version.hpp>

#include <ros/ros.h>
#include <ros/param.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <image_proc/advertisement_checker.h>
#include <image_transport/image_transport.h>
#include <visp_tracker/Init.h>
#include <visp_tracker/ModelBasedSettingsConfig.h>

#include <visp/vpMe.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpPose.h>

#define protected public
#include <visp/vpMbEdgeTracker.h>
#include <visp/vpMbKltTracker.h>
#include <visp/vpMbEdgeKltTracker.h>
#undef protected

#include <visp/vpDisplayX.h>

#include "conversion.hh"
#include "callbacks.hh"
#include "file.hh"
#include "names.hh"

#include "tracker-client.hh"


namespace visp_tracker
{
  TrackerClient::TrackerClient(ros::NodeHandle& nh,
			       ros::NodeHandle& privateNh,
			       volatile bool& exiting,
			       unsigned queueSize)
    : exiting_ (exiting),
      queueSize_(queueSize),
      nodeHandle_(nh),
      nodeHandlePrivate_(privateNh),
      imageTransport_(nodeHandle_),
      image_(),
      modelPath_(),
      modelName_(),
      cameraPrefix_(),
      rectifiedImageTopic_(),
      cameraInfoTopic_(),
      vrmlPath_(),
      initPath_(),
      cameraSubscriber_(),
      mutex_(),
      reconfigureSrv_(mutex_, nodeHandlePrivate_),
      movingEdge_(),
      kltTracker_(),
      cameraParameters_(),
      tracker_(),
      startFromSavedPose_(),
      checkInputs_(),
      resourceRetriever_()
  {
    // Parameters.
    nodeHandlePrivate_.param<std::string>("model_path", modelPath_,
					  visp_tracker::default_model_path);
    nodeHandlePrivate_.param<std::string>("model_name", modelName_, "");

    nodeHandlePrivate_.param<bool>
      ("start_from_saved_pose", startFromSavedPose_, false);

    nodeHandlePrivate_.param<bool>
      ("confirm_init", confirmInit_, true);

    nodeHandlePrivate_.param<std::string>("tracker_type", trackerType_, "mbt");
    if(trackerType_=="mbt")
      tracker_ = new vpMbEdgeTracker();
    else if(trackerType_=="klt")
      tracker_ = new vpMbKltTracker();
    else
      tracker_ = new vpMbEdgeKltTracker();

    //tracker_->resetTracker(); // TO CHECK

    if (modelName_.empty ())
      throw std::runtime_error
	("empty model\n"
	 "Relaunch the client while setting the model_name parameter, i.e.\n"
	 "$ rosrun visp_tracker client _model_name:=my-model"
	 );

    // Compute topic and services names.

    ros::Rate rate (1);
    while (cameraPrefix_.empty ())
      {
	if (!nodeHandle_.getParam ("camera_prefix", cameraPrefix_))
	  {
	    ROS_WARN
	      ("the camera_prefix parameter does not exist.\n"
	       "This may mean that:\n"
	       "- the tracker is not launched,\n"
	       "- the tracker and viewer are not running in the same namespace."
	       );
	  }
	else if (cameraPrefix_.empty ())
	  {
	    ROS_INFO
	      ("tracker is not yet initialized, waiting...\n"
	       "You may want to launch the client to initialize the tracker.");
	  }
	if (this->exiting())
	  return;
	rate.sleep ();
      }

    rectifiedImageTopic_ =
      ros::names::resolve(cameraPrefix_ + "/image_rect");
    cameraInfoTopic_ =
      ros::names::resolve(cameraPrefix_ + "/camera_info");

    // Check for subscribed topics.
    checkInputs();

    // Set callback for dynamic reconfigure.
    if(trackerType_!="klt"){
      vpMbEdgeTracker* t = dynamic_cast<vpMbEdgeTracker*>(tracker_);
      t->setMovingEdge(movingEdge_);
    }
    
    if(trackerType_!="mbt"){
      vpMbKltTracker* t = dynamic_cast<vpMbKltTracker*>(tracker_);
      t->setKltOpencv(kltTracker_);
    }
    
    // Dynamic reconfigure.
    reconfigureSrv_t::CallbackType f =
      boost::bind(&reconfigureCallback, boost::ref(tracker_),
                  boost::ref(image_), boost::ref(movingEdge_), boost::ref(kltTracker_),
                  boost::ref(trackerType_), boost::ref(mutex_), _1, _2);
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
    // if (!boost::filesystem::is_regular_file(vrmlPath_))
    //   {
    // 	boost::format fmt("VRML model %1% is not a regular file");
    // 	fmt % vrmlPath_;
    // 	throw std::runtime_error(fmt.str());
    //   }
    // if (!boost::filesystem::is_regular_file(initPath_))
    //   {
    // 	boost::format fmt("Initialization file %1% is not a regular file");
    // 	fmt % initPath_;
    // 	throw std::runtime_error(fmt.str());
    //   }

    // Load the 3d model.
    loadModel();

    // Wait for the image to be initialized.
    waitForImage();
    if (this->exiting())
      return;
    if (!image_.getWidth() || !image_.getHeight())
      throw std::runtime_error("failed to retrieve image");

    // Tracker initialization.
    // - Camera
    initializeVpCameraFromCameraInfo(cameraParameters_, info_);
    tracker_->setCameraParameters(cameraParameters_);
    tracker_->setDisplayFeatures(true);

    // - Moving edges.
    movingEdge_.initMask();
    if(trackerType_!="klt"){
      vpMbEdgeTracker* t = dynamic_cast<vpMbEdgeTracker*>(tracker_);
      t->setMovingEdge(movingEdge_);
    }
    
    if(trackerType_!="mbt"){
      vpMbKltTracker* t = dynamic_cast<vpMbKltTracker*>(tracker_);
      t->setKltOpencv(kltTracker_);
    }

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
    boost::format fmtWindowTitle ("ViSP MBT tracker initialization - [ns: %s]");
    fmtWindowTitle % ros::this_node::getNamespace ();

    vpDisplayX d(image_, image_.getWidth(), image_.getHeight(),
		 fmtWindowTitle.str().c_str());

    ros::Rate loop_rate_tracking(200);
    bool ok = false;
    vpHomogeneousMatrix cMo;
    vpImagePoint point (10, 10);
    while (!ok && !exiting())
      {
	try
	  {
	    // Initialize.
	    vpDisplay::display(image_);
	    vpDisplay::flush(image_);
	    if (!startFromSavedPose_)
	      init();
	    else
	      {
		cMo = loadInitialPose();
		startFromSavedPose_ = false;
        tracker_->initFromPose(image_, cMo);
	      }
        tracker_->getPose(cMo);

	    ROS_INFO_STREAM("initial pose [tx,ty,tz,tux,tuy,tuz]:\n"
			    << vpPoseVector(cMo));

	    // Track once to make sure initialization is correct.
	    if (confirmInit_)
	      {
		vpImagePoint ip;
		vpMouseButton::vpMouseButtonType button =
		  vpMouseButton::button1;
		do
		  {
		    vpDisplay::display(image_);
            tracker_->track(image_);
            tracker_->display(image_, cMo, cameraParameters_,
				     vpColor::red, 2);
		    vpDisplay::displayCharString
		      (image_, point, "tracking, click to initialize tracker",
		       vpColor::red);
		    vpDisplay::flush(image_);
            tracker_->getPose(cMo);

		    ros::spinOnce();
		    loop_rate_tracking.sleep();
		    if (exiting())
		      return;
		  }
		while(!vpDisplay::getClick(image_, ip, button, false));
		ok = true;
	      }
	    else
	      ok = true;
	  }
	catch(const std::runtime_error& e)
	  {
	    ROS_ERROR_STREAM("failed to initialize: "
			     << e.what() << ", retrying...");
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
    try
      {
	sendcMo(cMo);
      }
    catch(std::exception& e)
      {
	ROS_ERROR_STREAM("failed to send cMo: " << e.what ());
      }
    catch(...)
      {
	ROS_ERROR("unknown error happened while sending the cMo, retrying...");
      }
  }
  
  TrackerClient::~TrackerClient()
  {
    delete tracker_;
  }

  void
  TrackerClient::sendcMo(const vpHomogeneousMatrix& cMo)
  {
    ros::ServiceClient client =
      nodeHandle_.serviceClient<visp_tracker::Init>(visp_tracker::init_service);
    visp_tracker::Init srv;

    // Load the model and send it to the parameter server.
    std::string modelDescription = fetchResource
      (getModelFileFromModelName (modelName_, modelPath_));
    nodeHandle_.setParam (model_description_param, modelDescription);

    vpHomogeneousMatrixToTransform(srv.request.initial_cMo, cMo);
    
    if(trackerType_!="klt"){
      convertVpMeToInitRequest(movingEdge_, tracker_, srv);
    }
    
    if(trackerType_!="mbt"){
      convertVpKltOpencvToInitRequest(kltTracker_, tracker_, srv);
    }

    ros::Rate rate (1);
    while (!client.waitForExistence ())
      {
	ROS_INFO
	  ("Waiting for the initialization service to become available.");
	rate.sleep ();
      }

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
			 << vrmlPath_.native());

	std::string modelPath;
	boost::filesystem::ofstream modelStream;
	if (!makeModelFile(modelStream,
			   vrmlPath_.native(),
			   modelPath))
	  throw std::runtime_error ("failed to retrieve model");

    tracker_->loadModel(modelPath.c_str());
	ROS_INFO("VRML model has been successfully loaded.");

  if(trackerType_=="mbt"){
    vpMbEdgeTracker* t = dynamic_cast<vpMbEdgeTracker*>(tracker_);
    ROS_DEBUG_STREAM("Nb faces: "
                     << t->getFaces().getPolygon().size());
    ROS_DEBUG_STREAM("Nb visible faces: " << t->getFaces().getNbVisiblePolygon());

    std::list<vpMbtDistanceLine *> linesList;
    t->getLline(linesList);
    ROS_DEBUG_STREAM("Nb line: " << linesList.size());
    ROS_DEBUG_STREAM("nline: " << t->nline);
  }
  else if(trackerType_=="klt"){
    vpMbKltTracker* t = dynamic_cast<vpMbKltTracker*>(tracker_);
    ROS_DEBUG_STREAM("Nb faces: "
                     << t->getFaces().getPolygon().size());
    ROS_DEBUG_STREAM("Nb visible faces: " << t->getFaces().getNbVisiblePolygon());
    ROS_DEBUG_STREAM("Nb KLT points: " << t->getNbKltPoints());
  }
  else {
    vpMbEdgeKltTracker* t = dynamic_cast<vpMbEdgeKltTracker*>(tracker_);
    ROS_DEBUG_STREAM("Nb hidden faces: "
                     << t->getFaces().getPolygon().size());
    ROS_DEBUG_STREAM("Nb visible faces: " << t->getFaces().getNbVisiblePolygon());
    ROS_DEBUG_STREAM("Nb KLT points: " << t->getNbKltPoints());

    std::list<vpMbtDistanceLine *> linesList;
    t->getLline(linesList);
    ROS_DEBUG_STREAM("Nb line: " << linesList.size());
    ROS_DEBUG_STREAM("nline: " << t->nline);
  }
      }
    catch(...)
      {
	boost::format fmt
	  ("Failed to load the model %1%\n"
	   "Do you use resource_retriever syntax?\n"
	   "I.e. replace /my/model/path by file:///my/model/path");
	fmt % vrmlPath_;
	throw std::runtime_error(fmt.str());
      }
  }

  vpHomogeneousMatrix
  TrackerClient::loadInitialPose()
  {
    vpHomogeneousMatrix cMo;
    cMo.eye();

    std::string initialPose =
      getInitialPoseFileFromModelName (modelName_, modelPath_);
    std::string resource;
    try
      {
	resource = fetchResource (initialPose);
      }
    catch (...)
      {
	ROS_WARN_STREAM
	  ("failed to retrieve initial pose: " << initialPose << "\n"
	   << "using identity as initial pose");
	return cMo;
      }
    std::stringstream file;
    file << resource;

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
	ROS_WARN_STREAM
	  ("failed to save initial pose: " << initialPose
	   <<"\n"
	   <<"Note: this is normal is you use a remote resource."
	   <<"\n"
	   <<"I.e. your model path starts with http://, package://, etc.");
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

    std::string init =
      getInitFileFromModelName(modelName_, modelPath_);
    std::string resource = fetchResource(init);
    std::stringstream file;
    file << resource;

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

  bool
  TrackerClient::validatePose(const vpHomogeneousMatrix &cMo){
    ros::Rate loop_rate(200);
    vpImagePoint ip;
    vpMouseButton::vpMouseButtonType button = vpMouseButton::button1;

    vpDisplay::display(image_);
    tracker_->display(image_, cMo, cameraParameters_, vpColor::green);
    vpDisplay::displayFrame(image_, cMo, cameraParameters_,0.05, vpColor::green);
    vpDisplay::displayCharString(image_, 15, 10,
        "left click to validate, right click to modify initial pose",
        vpColor::red);
    vpDisplay::flush(image_);

    do
    {
      ros::spinOnce();
      loop_rate.sleep();
      if (!ros::ok())
        return false;
    }
    while(ros::ok() && !vpDisplay::getClick(image_, ip, button, false));

    if(button == vpMouseButton::button1)
      return true;

    return false;
  }

  void
  TrackerClient::init()
  {
    ros::Rate loop_rate(200);
    vpHomogeneousMatrix cMo;
    vpImagePoint point (10, 10);

    cMo = loadInitialPose();

    // Show last cMo.
    vpImagePoint ip;
    vpMouseButton::vpMouseButtonType button = vpMouseButton::button1;

    if(validatePose(cMo))
    {
      tracker_->initFromPose(image_, cMo);
      return;
    }

    points_t points = loadInitializationPoints();
    imagePoints_t imagePoints;

    bool done = false;
    while(!done){
      vpDisplay::display(image_);
      vpDisplay::flush(image_);

      imagePoints.clear();
      for(unsigned i = 0; i < points.size(); ++i)
      {
        do
        {
          ros::spinOnce();
          loop_rate.sleep();
          if (!ros::ok())
            return;
        }
        while(ros::ok() && !vpDisplay::getClick(image_, ip, button, false));

        imagePoints.push_back(ip);
        vpDisplay::displayCross(image_, imagePoints.back(), 5, vpColor::green);
        vpDisplay::flush(image_);
      }

      tracker_->initFromPoints(image_,imagePoints,points);
      tracker_->getPose(cMo);
      if(validatePose(cMo))
        done = true;
    }
    tracker_->initFromPose(image_, cMo);
    saveInitialPose(cMo);
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
	if (exiting())
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
  TrackerClient::waitForImage()
  {
    ros::Rate loop_rate(10);
    while (!exiting()
	   && (!image_.getWidth() || !image_.getHeight()))
      {
	ROS_INFO_THROTTLE(1, "waiting for a rectified image...");
	ros::spinOnce();
	loop_rate.sleep();
      }
  }

  std::string
  TrackerClient::fetchResource(const std::string& s)
  {
    resource_retriever::MemoryResource resource =
      resourceRetriever_.get(s);
    std::string result;
    result.resize(resource.size);
    unsigned i = 0;
    for (; i < resource.size; ++i)
      result[i] = resource.data.get()[i];
    return result;
  }

  bool
  TrackerClient::makeModelFile(boost::filesystem::ofstream& modelStream,
			       const std::string& resourcePath,
			       std::string& fullModelPath)
  {
    resource_retriever::MemoryResource resource =
      resourceRetriever_.get(resourcePath);
    std::string result;
    result.resize(resource.size);
    unsigned i = 0;
    for (; i < resource.size; ++i)
      result[i] = resource.data.get()[i];
    result[resource.size];

    char* tmpname = strdup("/tmp/tmpXXXXXX");
    if (mkdtemp(tmpname) == NULL)
      {
	ROS_ERROR_STREAM
	  ("Failed to create the temporary directory: " << strerror(errno));
	return false;
      }
    boost::filesystem::path path(tmpname);
    path /= "model.wrl";
    free(tmpname);

    fullModelPath = path.native();

    modelStream.open(path);
    if (!modelStream.good())
      {
	ROS_ERROR_STREAM
	  ("Failed to create the temporary file: " << path);
	return false;
      }
    modelStream << result;
    modelStream.flush();
    return true;
  }
} // end of namespace visp_tracker.
