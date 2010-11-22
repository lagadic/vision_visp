#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <ros/param.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>

#include <visp_tracker/TrackingResult.h>
#include <visp_tracker/Init.h>

#include <boost/bind.hpp>
#include <visp/vpImage.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpMbEdgeTracker.h>

typedef vpImage<unsigned char> image_t;

enum State
  {
    WAITING_FOR_INITIALIZATION,
    TRACKING,
    LOST
  };

//FIXME: to be split into rosImageToVisp.
void imageCallback(image_t& image, const sensor_msgs::Image::ConstPtr& msg,
		   const sensor_msgs::CameraInfoConstPtr& info)
{
  // For now, we suppose that mono8 is used.
  if(msg->encoding != "mono8")
    {
      ROS_ERROR("Bad encoding `%s', dropping the frame.",
		msg->encoding.c_str());
      return;
    }

  // Resize the image if necessary, this should only happen once.
  if (msg->width != image.getWidth() || msg->height != image.getHeight())
    {
      ROS_INFO
	("Received image is %dx%d but ViSP image size is %dx%d, resizing.",
	 msg->width, msg->height,
	 image.getWidth (), image.getHeight ());
      image.resize (msg->height, msg->width);
    }

  for(unsigned i = 0; i < image.getWidth (); ++i)
    for(unsigned j = 0; j < image.getHeight (); ++j)
      image[j][i] = msg->data[j * msg->step + i];
}

bool initCallback(State& state,
		  vpMbEdgeTracker& tracker,
		  image_t& image,
		  visp_tracker::Init::Request& req,
		  visp_tracker::Init::Response& res)
{
  vpHomogeneousMatrix cMo;
  cMo.eye(); //FIXME: parse req for that.

  res.initialization_succeed = true;
  state = TRACKING;
  try
    {
      tracker.init(image, cMo);
    }
  catch(...)
    {
      state = WAITING_FOR_INITIALIZATION;
      res.initialization_succeed = false;
    }
  return true;
}

int main(int argc, char **argv)
{
  State state = WAITING_FOR_INITIALIZATION;
  std::string image_topic;
  std::string model_path;

  image_t I;

  ros::init(argc, argv, "tracker_mbt");

  ros::NodeHandle n("tracker_mbt");
  image_transport::ImageTransport it(n);

  // Parameters.
  ros::param::param<std::string>("~image", image_topic, "/camera/image_raw");
  ros::param::param<std::string>("~model_path", model_path, "");

  // Result publisher.
  ros::Publisher result_pub =
    n.advertise<visp_tracker::TrackingResult>("result", 1000);

  // Output publisher.
  image_transport::Publisher output_pub = it.advertise("output", 1);

  // Camera subscriber.
  image_transport::CameraSubscriber::Callback image_callback
    (boost::bind(imageCallback, boost::ref(I), _1, _2));
  image_transport::CameraSubscriber sub =
    it.subscribeCamera(image_topic, 1, image_callback);

  // Initialization.
  ros::Rate loop_rate(10);

  vpMbEdgeTracker tracker;

  // Model loading.
  ros::param::get("model_path", model_path);
  try
    {
      ROS_DEBUG("Trying to load the model `%s'.", model_path.c_str());
      tracker.loadModel(model_path.c_str());
    }
  catch(...)
    {
      ROS_ERROR("Failed to load the model `%s'.", model_path.c_str());
      return 1;
    }
  ROS_DEBUG("Model has been successfully loaded.");

  // Service declaration.
  boost::function<bool (visp_tracker::Init::Request&,
			visp_tracker::Init::Response& res)>
  init_callback (boost::bind(initCallback, boost::ref(state),
			     boost::ref(tracker),
			     boost::ref(I), _1, _2));
  ros::ServiceServer service = n.advertiseService("init", init_callback);

  // Tracker initialization.
  //FIXME: replace by real camera parameters of the rectified camera.
  vpCameraParameters cam(320, 240, 0.1, 0.1);
  tracker.setCameraParameters(cam);
  tracker.setDisplayMovingEdges(true);

  // Main loop.
  while (ros::ok())
    {
      vpHomogeneousMatrix cMo;
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
      visp_tracker::TrackingResult result;
      result.is_tracking = state == TRACKING;
      if (state == TRACKING)
	{
	  //FIXME: to be done result.cMo.header
	  //FIXME: to be done result.cMo.child_frame_id
	  result.cMo.transform.translation.x = cMo[0][3];
	  result.cMo.transform.translation.y = cMo[1][3];
	  result.cMo.transform.translation.z = cMo[2][3];
	  //FIXME: to be done result.cMo.transform.rotation.x
	  //FIXME: to be done result.cMo.transform.rotation.y
	  //FIXME: to be done result.cMo.transform.rotation.z
	  //FIXME: to be done result.cMo.transform.rotation.w
	}
      result_pub.publish(result);

      // Convert Visp image into ROS image.
      sensor_msgs::Image image;
      //FIXME: fill header.
      image.width = I.getWidth();
      image.height = I.getHeight();
      image.encoding = "mono8";
      image.step = I.getWidth();
      image.data.resize(image.height * image.step);
      for(unsigned i = 0; i < I.getWidth (); ++i)
      	for(unsigned j = 0; j < I.getHeight (); ++j)
      	  image.data[j * image.step + i] = I[j][i];

      output_pub.publish(image);


      ros::spinOnce();
      loop_rate.sleep();
    }

  return 0;
}
