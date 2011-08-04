#include <cstdlib>
#include <fstream>

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/format.hpp>
#include <boost/optional.hpp>

#include <ros/ros.h>
#include <ros/param.h>
#include <dynamic_reconfigure/server.h>
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

typedef vpImage<unsigned char> image_t;

vpHomogeneousMatrix loadInitialPose(const std::string& model_path,
				    const std::string& model_name)
{
  vpHomogeneousMatrix cMo;
  cMo.eye();

  boost::filesystem::path initial_pose =
    getInitialPoseFileFromModelName(model_name, model_path);
  boost::filesystem::ifstream file(initial_pose);
  if (!file.good())
    {
      ROS_WARN_STREAM("failed to load initial pose: " << initial_pose << "\n"
		      << "using identity as initial pose");
      return cMo;
    }

  vpPoseVector pose;
  for (unsigned i = 0; i < 6; ++i)
    if (file.good())
      file >> pose[i];
    else
      {
	std::cout << i << std::endl;
	ROS_WARN("failed to parse initial pose file");
	cMo.eye();
	return cMo;
      }
  cMo.buildFrom(pose);
  return cMo;
}

void saveInitialPose(const vpHomogeneousMatrix& cMo,
		     const std::string& model_path,
		     const std::string& model_name)
{
  boost::filesystem::path initial_pose =
    getInitialPoseFileFromModelName(model_name, model_path);
  boost::filesystem::ofstream file(initial_pose);
  if (!file.good())
    {
      ROS_WARN_STREAM("failed to save initial pose: " << initial_pose);
      return;
    }
  
  vpPoseVector pose;
  pose.buildFrom(cMo);
  file << pose;
}

std::vector<vpPoint> loadInitializationPoints(const std::string& model_path,
					      const std::string& model_name)
{
  std::vector<vpPoint> points;

  boost::filesystem::path init =
    getInitFileFromModelName(model_name, model_path);
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

void initPoint(unsigned& i,
	       std::vector<vpPoint>& points,
	       std::vector<vpImagePoint>& imagePoints,
	       ros::Rate& rate, image_t& I,
	       vpPose& pose, vpCameraParameters& cam)
{
  vpImagePoint ip;
  double x = 0., y = 0.;
  boost::format fmt("click on point %u/%u");
  fmt % (i + 1) % points.size(); // list points from 1 to n.

  // Click on the point.
  vpMouseButton::vpMouseButtonType button = vpMouseButton::button1;
  do
    {
      vpDisplay::display(I);
      vpDisplay::displayCharString
	(I, 15, 10,
	 fmt.str().c_str(),
	 vpColor::red);

      for (unsigned j = 0; j < imagePoints.size(); ++j)
	vpDisplay::displayCross(I, imagePoints[j], 5, vpColor::green);

      vpDisplay::flush(I);
      ros::spinOnce();
      rate.sleep();
      if (!ros::ok())
	exit(1);
    }
  while(!vpDisplay::getClick(I, ip, button, false));

  imagePoints.push_back(ip);
  vpPixelMeterConversion::convertPoint(cam, ip, x, y);
  points[i].set_x(x);
  points[i].set_y(y);
  pose.addPoint(points[i]);
}

void initClick(vpMbEdgeTracker& tracker,
	       image_t& I,
	       vpCameraParameters& cam,
	       const std::string& model_path,
	       const std::string& model_name)
{
  ros::Rate loop_rate(200);
  vpHomogeneousMatrix cMo;
  vpImagePoint point (10, 10);

  cMo = loadInitialPose(model_path, model_name);

  // Show last cMo.
  vpImagePoint ip;
  vpMouseButton::vpMouseButtonType button = vpMouseButton::button1;
  do
    {
      vpDisplay::display(I);
      tracker.display(I, cMo, cam, vpColor::green);
      vpDisplay::displayFrame(I, cMo, cam, 0.05, vpColor::green);
      vpDisplay::displayCharString
	(I, 15, 10,
	 "left click to validate, right click to modify initial pose",
	 vpColor::red);
      vpDisplay::flush(I);
      ros::spinOnce();
      loop_rate.sleep();
      if (!ros::ok())
	exit(1);
    }
  while(!vpDisplay::getClick(I, ip, button, false));

  if(button == vpMouseButton::button1)
    {
      tracker.init(I, cMo);
      return;
    }

  std::vector<vpPoint> points =
    loadInitializationPoints(model_path, model_name);
  std::vector<vpImagePoint> imagePoints;

  vpPose pose;
  pose.clearPoint();
  bool done = false;
  while (!done)
    {
      // Initialize points.
      for(unsigned i = 0; i < points.size(); ++i)
	initPoint(i, points, imagePoints, loop_rate, I, pose, cam);

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
	  vpDisplay::display(I);
	  tracker.display(I, cMo, cam, vpColor::green);
	  vpDisplay::displayCharString
	    (I, 15, 10,
	     "left click to validate, right click to re initialize object",
	     vpColor::red);
	  vpDisplay::flush(I);
	  ros::spinOnce();
	  loop_rate.sleep();
	  if (!ros::ok())
	    exit(1);
	}
      while(!vpDisplay::getClick(I, ip, button, false));

      if(button != vpMouseButton::button1)
	{
	  pose.clearPoint();
	  imagePoints.clear();
	}
      else
	done = true;
    }
  tracker.init(I, cMo);
  saveInitialPose(cMo, model_path, model_name);
}

int main(int argc, char **argv)
{
  std::string camera_prefix;
  std::string tracker_prefix;
  std::string model_path;
  std::string model_name;
  vpMe moving_edge;

  image_t I;

  vpMbEdgeTracker tracker;
  tracker.resetTracker();

  // Initialization.
  ros::init(argc, argv, "tracker_mbt_client");

  ros::NodeHandle n;
  image_transport::ImageTransport it(n);

  // Parameters.
  ros::param::param<std::string>("~camera_prefix", camera_prefix, "");
  ros::param::param<std::string>("~tracker_prefix",
				 tracker_prefix,
				 visp_tracker::default_tracker_prefix);

  ros::param::param<std::string>("~model_path", model_path,
				 visp_tracker::default_model_path);
  ros::param::param<std::string>("~model_name", model_name, "");

  // Compute topic and services names.
  const std::string rectified_image_topic =
    ros::names::clean(camera_prefix + "/image_rect");
  const std::string camera_info_topic =
    ros::names::clean(camera_prefix + "/camera_info");

  const std::string init_service =
    ros::names::clean(tracker_prefix + "/" + visp_tracker::init_service);

  // Dynamic reconfigure.
  dynamic_reconfigure::Server<visp_tracker::MovingEdgeConfig> reconfigureSrv;
  dynamic_reconfigure::Server<visp_tracker::MovingEdgeConfig>::CallbackType f;
  f = boost::bind(&reconfigureCallback, boost::ref(tracker),
		  boost::ref(I), boost::ref(moving_edge), _1, _2);
  reconfigureSrv.setCallback(f);

  // Camera subscriber.
  std_msgs::Header header;
  sensor_msgs::CameraInfoConstPtr info;
  image_transport::CameraSubscriber sub =
    it.subscribeCamera(rectified_image_topic, 100,
		       bindImageCallback(I, header, info));

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

  // Wait for the image to be initialized.
  ros::Rate loop_rate(10);
  while (!I.getWidth() || !I.getHeight())
    {
      ros::spinOnce();
      loop_rate.sleep();
      if (!ros::ok())
	exit(1);
    }

  // Tracker initialization.

  // - Camera
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

  // - Moving edges.
  moving_edge.initMask();
  tracker.setMovingEdge(moving_edge);

  // Display camera parameters and moving edges settings.
  ROS_INFO_STREAM(cam);
  moving_edge.print();

  vpDisplayX d(I, I.getWidth(), I.getHeight(),
	       "ViSP MBT tracker initialization");

  ros::Rate loop_rate_tracking(200);
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
	  initClick(tracker, I, cam, model_path, model_name);
	  tracker.getPose(cMo);

	  ROS_INFO_STREAM("initial pose [tx,ty,tz,tux,tuy,tuz]:\n"
			  << vpPoseVector(cMo));

	  // Track once to make sure initialization is correct.
	  vpImagePoint ip;
	  vpMouseButton::vpMouseButtonType button = vpMouseButton::button1;
	  do
	    {
	      vpDisplay::display(I);
	      tracker.track(I);
	      tracker.display(I, cMo, cam, vpColor::red, 2);
	      vpDisplay::displayCharString
		(I, point, "tracking, click to initialize tracker",
		 vpColor::red);
	      vpDisplay::flush(I);
	      tracker.getPose(cMo);

	      ros::spinOnce();
	      loop_rate_tracking.sleep();
	      if (!ros::ok())
		exit(1);
	    }
	  while(!vpDisplay::getClick(I, ip, button, false));
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
  vpHomogeneousMatrixToTransform(srv.request.initial_cMo, cMo);

  convertVpMeToInitRequest(moving_edge, tracker, srv);

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
