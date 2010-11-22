#include <cstdlib>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <ros/param.h>
#include <visp_tracker/Init.h>

#include <visp/vpMbEdgeTracker.h>

typedef vpImage<unsigned char> image_t;

int main(int argc, char **argv)
{
  std::string image_topic;
  std::string model_path;

  image_t I;


  ros::init(argc, argv, "tracker_mbt_client");

  ros::NodeHandle n;
  image_transport::ImageTransport it(n);

  // Parameters.
  ros::param::param<std::string>("~image", image_topic, "/camera/image_raw");
  ros::param::param<std::string>("~model_path", model_path, "");

  // Camera subscriber.
  /*
  image_transport::CameraSubscriber::Callback image_callback
    (boost::bind(imageCallback, boost::ref(I), _1, _2));
  image_transport::CameraSubscriber sub =
    it.subscribeCamera(image_topic, 1, image_callback);
  */

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

  // Tracker initialization.
  //FIXME: replace by real camera parameters of the rectified camera.
  vpCameraParameters cam(320, 240, 0.1, 0.1);
  tracker.setCameraParameters(cam);

  // FIXME: wait for the image to be initialized.

  tracker.initClick(I, "FIXME: 3d points file");

  vpHomogeneousMatrix cMo;
  tracker.getPose(cMo);


  ros::ServiceClient client =
    n.serviceClient<visp_tracker::Init>("init");
  visp_tracker::Init srv;

srv.request.initial_cMo.translation.x = cMo[0][3];
srv.request.initial_cMo.translation.y = cMo[1][3];
srv.request.initial_cMo.translation.z = cMo[2][3];
//FIXME: to be done srv.request.initial_cMo.rotation.x
//FIXME: to be done srv.request.initial_cMo.rotation.y
//FIXME: to be done srv.request.initial_cMo.rotation.z
//FIXME: to be done srv.request.initial_cMo.rotation.w

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
