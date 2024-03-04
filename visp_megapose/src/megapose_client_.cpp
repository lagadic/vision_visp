#include <rclcpp/rclcpp.hpp>

// visp includes
#include <visp3/core/vpTime.h>
#include <visp3/gui/vpDisplayX.h>

// visp bridge includes
#include <visp_bridge/3dpose.h>
#include <visp_bridge/camera.h>
#include <visp_bridge/image.h>

// ROS2 includes
#include "tf2_ros/transform_broadcaster.h"
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

// ROS2 message filter includes
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

// visp MegaPose tag detector includes
#include <visp3/dnn_tracker/vpMegaPose.h>
#include <visp3/dnn_tracker/vpMegaPoseTracker.h>
#include <visp3/io/vpJsonArgumentParser.h>
#include <nlohmann/json.hpp>
  enum DetectionMethod
    {
    UNKNOWN,
    CLICK,
    DNN
    };

NLOHMANN_JSON_SERIALIZE_ENUM(DetectionMethod, {
  {UNKNOWN, nullptr}, // Default value if the json string is not in "current", "desired" or "mean"
  {CLICK, "click"},
  {DNN, "dnn"} }
);

class MegaPoseClient : public rclcpp::Node
{
private:
  unsigned long queue_size_;

  image_transport::SubscriberFilter raw_image_subscriber;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> camera_info_subscriber;

  using SyncPolicy =
      message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;
  std::shared_ptr<Synchronizer> synchronizer_;

  bool debug_display_;

  vpImage<vpRGBa> I_; // Image used for debug display, type unsigned char is grayscale, type vpRGBa is color
  std_msgs::msg::Header image_header_;
  bool got_image_;
  vpCameraParameters cam_;
  unsigned width_, height_;

  void waitForImage();

  void frameCallback(const sensor_msgs::msg::Image::ConstSharedPtr &image,
                     const sensor_msgs::msg::CameraInfo::ConstSharedPtr &cam_info);
  void broadcastTransform(const vpHomogeneousMatrix &cMo, std::string &child_frame_id);
  
  vpColor interpolate(const vpColor &low, const vpColor &high, const float f);
  void displayScore(float);
  std::optional<vpRect> detectObjectForInitMegaposeClick();
  void overlayRender(const vpImage<vpRGBa> &overlay);

public:
  MegaPoseClient();
  void spin();
};

MegaPoseClient::MegaPoseClient()
  : Node("MegaPoseClient"),

    queue_size_(1000), debug_display_(false), I_(), image_header_(), got_image_(false), cam_()
{
  debug_display_ = true;
}

void MegaPoseClient::waitForImage()
{
  rclcpp::Rate loop_rate(10);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for a rectified image...");
  while (rclcpp::ok()) {
    if (got_image_) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Got image!");
      return;
    }
    rclcpp::spin_some(this->get_node_base_interface());
    loop_rate.sleep();
  }
}
void MegaPoseClient::frameCallback(const sensor_msgs::msg::Image::ConstSharedPtr &image,
                                   const sensor_msgs::msg::CameraInfo::ConstSharedPtr &cam_info)
{
  image_header_ = image->header;
  I_ = visp_bridge::toVispImageRGBa(*image);
  cam_ = visp_bridge::toVispCameraParameters(*cam_info);
  width_ = image->width;
  height_ = image->height;

  got_image_ = true;
}
void MegaPoseClient::broadcastTransform(const vpHomogeneousMatrix &cMo, std::string &child_frame_id)
{
  static auto tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  static geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped.header.stamp = this->get_clock()->now();
  transformStamped.header.frame_id = "camera_color_optical_frame";
  transformStamped.child_frame_id = child_frame_id;
  transformStamped.transform = visp_bridge::toGeometryMsgsTransform(cMo);
  tf_broadcaster_->sendTransform(transformStamped);
}
void MegaPoseClient::spin()
{
  std::string image_topic = "/camera/camera/color/image_raw";
  std::string camera_info_topic = "/camera/camera/color/camera_info";
  raw_image_subscriber.subscribe(this, image_topic, "raw");
  camera_info_subscriber.subscribe(this, camera_info_topic);
  synchronizer_ = std::make_shared<Synchronizer>(SyncPolicy(queue_size_), raw_image_subscriber, camera_info_subscriber);
  synchronizer_->registerCallback(
      std::bind(&MegaPoseClient::frameCallback, this, std::placeholders::_1, std::placeholders::_2));
  waitForImage();
  // MegaPoseClient
  std::string megaposeAddress = "127.0.0.1";
  unsigned megaposePort = 5555;
  std::string objectName = "cube";
  int refinerIterations = 1, coarseNumSamples = 576;
  double reinitThreshold = 0.2;
  DetectionMethod detectionMethod = DetectionMethod::CLICK;
  std::shared_ptr<vpMegaPose> megapose;
  try {
    megapose = std::make_shared<vpMegaPose>(megaposeAddress, megaposePort, cam_, height_, width_);
  } catch (...) {
    throw vpException(vpException::ioError, "Could not connect to Megapose server at " + megaposeAddress + " on port " +
                                                std::to_string(megaposePort));
  }
  vpMegaPoseTracker megaposeTracker(megapose, objectName, refinerIterations);
  megapose->setCoarseNumSamples(coarseNumSamples);
  const std::vector<std::string> allObjects = megapose->getObjectNames();
  if (std::find(allObjects.begin(), allObjects.end(), objectName) == allObjects.end()) {
    throw vpException(vpException::badValue, "Object " + objectName + " is not known by the Megapose server!");
  }
  std::future<vpMegaPoseEstimate> trackerFuture;
  vpMegaPoseEstimate megaposeEstimate; // last Megapose estimation
  vpRect lastDetection; // Last detection (initialization)
  bool callMegapose = true; // Whether we should call Megapose this iteration
  bool initialized = false; // Whether tracking should be initialized or reinitialized
  bool tracking = false;

  bool overlayModel = true;
  vpImage<vpRGBa> overlayImage(height_, width_);
  std::string overlayMode = "full";
  std::vector<double> megaposeTimes;
  std::vector<double> frameTimes;
  // MegaPoseClient

  vpDisplayX *d = NULL;
  if (debug_display_)
    d = new vpDisplayX();

  if (debug_display_) {
    d->init(I_); // also init display
    vpDisplay::setTitle(I_, "MegaPoseClient debug display");
  }
  // when an image is ready tell the tracker to start searching for patterns
  rclcpp::Rate rate(10);
  while (rclcpp::ok()) {
    rclcpp::spin_some(this->get_node_base_interface());
    if (!callMegapose && trackerFuture.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
      megaposeEstimate = trackerFuture.get();
      callMegapose = true;
      tracking = true;

      if (overlayModel) {
        overlayImage = megapose->viewObjects({ objectName }, { megaposeEstimate.cTo }, overlayMode);
      }

      if (megaposeEstimate.score < reinitThreshold) { // If confidence is low, require a reinitialisation with 2D detection
        initialized = false;
      }
    }
    if (callMegapose) {
      if (!initialized) {
        tracking = false;
        std::optional<vpRect> detection = std::nullopt;
// #if (VISP_HAVE_OPENCV_VERSION >= 0x030403) && defined(HAVE_OPENCV_DNN) && 
//     ((__cplusplus >= 201703L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201703L)))
//         if (detectionMethod == DetectionMethod::DNN) {
//           detection = detectObjectForInitMegaposeDnn(
//             dnn, frame, objectName, initialized ? std::optional(megaposeEstimate) : std::nullopt);
//         }
// #endif
        if (detectionMethod == DetectionMethod::CLICK) {
          detection = detectObjectForInitMegaposeClick();
        }

        if (detection) {
          initialized = true;
          lastDetection = *detection;
          trackerFuture = megaposeTracker.init(I_, lastDetection);
          callMegapose = false;

        }
      }
      else {
        trackerFuture = megaposeTracker.track(I_);
        callMegapose = false;
      }
    }
    //! [Call MegaPose]

    //! [Display]
    std::string keyboardEvent;
    const bool keyPressed = vpDisplay::getKeyboardEvent(I_, keyboardEvent, false);
    if (keyPressed) {
      if (keyboardEvent == "t") {
        overlayModel = !overlayModel;
      }
      else if (keyboardEvent == "w") {
        overlayMode = overlayMode == "full" ? "wireframe" : "full";
      }
    }

    if (tracking) {
      if (overlayModel) {
        overlayRender(overlayImage);
        vpDisplay::display(I_);
      }
      vpDisplay::displayText(I_, 20, 20, "Right click to quit", vpColor::red);
      vpDisplay::displayText(I_, 30, 20, "Press T: Toggle overlay", vpColor::red);
      vpDisplay::displayText(I_, 40, 20, "Press W: Toggle wireframe", vpColor::red);
      vpDisplay::displayFrame(I_, megaposeEstimate.cTo, cam_, 0.05, vpColor::none, 3);
      broadcastTransform(megaposeEstimate.cTo, objectName);
      //vpDisplay::displayRectangle(I_, lastDetection, vpColor::red);
      displayScore(megaposeEstimate.score);
    }
    //! [Display]

    vpDisplay::flush(I_);

    vpMouseButton::vpMouseButtonType button;
    if (vpDisplay::getClick(I_, button, false)) {
      if (button == vpMouseButton::button3) {
        break; // Right click to stop
      }
    }
    vpDisplay::display(I_);
    vpDisplay::flush(I_);
    rate.sleep();
  }
  if (debug_display_)
    delete d;
}
void MegaPoseClient::displayScore(float score)
{
  const unsigned top = static_cast<unsigned>(I_.getHeight() * 0.85f);
  const unsigned height = static_cast<unsigned>(I_.getHeight() * 0.1f);
  const unsigned left = static_cast<unsigned>(I_.getWidth() * 0.05f);
  const unsigned width = static_cast<unsigned>(I_.getWidth() * 0.5f);
  vpRect full(left, top, width, height);
  vpRect scoreRect(left, top, width * score, height);
  const vpColor low = vpColor::red;
  const vpColor high = vpColor::green;
  const vpColor c = interpolate(low, high, score);

  vpDisplay::displayRectangle(I_, full, c, false, 5);
  vpDisplay::displayRectangle(I_, scoreRect, c, true, 1);
}
vpColor MegaPoseClient::interpolate(const vpColor &low, const vpColor &high, const float f)
{
  const float r = ((float)high.R - (float)low.R) * f;
  const float g = ((float)high.G - (float)low.G) * f;
  const float b = ((float)high.B - (float)low.B) * f;
  return vpColor((unsigned char)r, (unsigned char)g, (unsigned char)b);
}
std::optional<vpRect> MegaPoseClient::detectObjectForInitMegaposeClick()
{
  const bool startLabelling = vpDisplay::getClick(I_, false);

  const vpImagePoint textPosition(10.0, 20.0);

  if (startLabelling) {
    vpImagePoint topLeft, bottomRight;
    vpDisplay::displayText(I_, textPosition, "Click the upper left corner of the bounding box", vpColor::red);
    vpDisplay::flush(I_);
    vpDisplay::getClick(I_, topLeft, true);
    vpDisplay::display(I_);
    vpDisplay::displayCross(I_, topLeft, 5, vpColor::red, 2);
    vpDisplay::displayText(I_, textPosition, "Click the bottom right corner of the bounding box", vpColor::red);
    vpDisplay::flush(I_);
    vpDisplay::getClick(I_, bottomRight, true);
    vpRect bb(topLeft, bottomRight);
    return bb;
  }
  else {
    vpDisplay::display(I_);
    vpDisplay::displayText(I_, textPosition, "Click when the object is visible and static to start reinitializing megapose.", vpColor::red);
    vpDisplay::flush(I_);
    return std::nullopt;
  }
}
void MegaPoseClient::overlayRender(const vpImage<vpRGBa> &overlay)
{
  const vpRGBa black = vpRGBa(0, 0, 0);
  for (unsigned int i = 0; i < height_; ++i) {
    for (unsigned int j = 0; j < width_; ++j) {
      if (overlay[i][j] != black) {
        I_[i][j] = overlay[i][j];
      }
    }
  }
}
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
    MegaPoseClient().spin();

  return 0;
}
