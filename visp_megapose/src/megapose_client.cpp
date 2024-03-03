#include <rclcpp/rclcpp.hpp>
#include <filesystem>  // C++17 filesystem library for checking file existence

// visp includes
#include <visp3/core/vpTime.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/detection/vpDetectorDNNOpenCV.h>

// OpenCV/visp bridge includes
#include <visp_bridge/3dpose.h>
#include <visp_bridge/camera.h>
#include <visp_bridge/image.h>
#include <opencv4/opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

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

// ROS2 visp_megapose Server includes
#include <visp_megapose/srv/init.hpp>
#include <visp_megapose/srv/track.hpp>
#include <visp_megapose/srv/render.hpp>

using namespace std::chrono_literals; // For using time literals like 1s

enum DetectionMethod
{
  UNKNOWN,
  CLICK,
  DNN
};

std::map<std::string, DetectionMethod> stringToDetectionMethod = {
    {"UNKNOWN", UNKNOWN},
    {"CLICK", CLICK},
    {"DNN", DNN}};

bool fileExists(const std::string &path)
{
  std::filesystem::path p{path};
  return std::filesystem::exists(p) && std::filesystem::is_regular_file(p);
}

class MegaPoseClient : public rclcpp::Node
{
private:
  image_transport::SubscriberFilter raw_image_subscriber;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> camera_info_subscriber;
  void frameCallback(const sensor_msgs::msg::Image::ConstSharedPtr &image,
                     const sensor_msgs::msg::CameraInfo::ConstSharedPtr &cam_info);
  bool got_image_;
  vpCameraParameters vpcam_info_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr roscam_info_;
  unsigned width_, height_;
  using SyncPolicy =
      message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;
  std::shared_ptr<Synchronizer> synchronizer_;

  vpImage<vpRGBa> vpI_;                          // Image used for debug display
  sensor_msgs::msg::Image::ConstSharedPtr rosI_; // Image received from ROS2

  double reinitThreshold_;
  double confidence_;

  void waitForImage();
  void initial_pose_service_response_callback(rclcpp::Client<visp_megapose::srv::Init>::SharedFuture future);
  bool initialized_;
  bool init_request_done_;
  void track_pose_service_response_callback(rclcpp::Client<visp_megapose::srv::Track>::SharedFuture future);
  bool track_request_done_;
  void render_service_response_callback(rclcpp::Client<visp_megapose::srv::Render>::SharedFuture future);
  bool render_request_done_;

  void broadcastTransform(const geometry_msgs::msg::Transform &transform, const std::string &child_frame_id, const std::string &camera_tf);
  geometry_msgs::msg::Transform transform_;

  vpColor interpolate(const vpColor &low, const vpColor &high, const float f);
  void displayScore(float);
  std::optional<vpRect> detectObjectForInitMegaposeClick();
  void overlayRender(const vpImage<vpRGBa> &overlay);
  vpImage<vpRGBa> overlay_img_;
  bool overlayModel_;

  std::optional<vpRect> detectObjectForInitMegaposeDnn(const std::string &detectionLabel);
  DetectionMethod getDetectionMethodFromString(const std::string &str);
  vpDetectorDNNOpenCV dnn_;

public:
  MegaPoseClient();
  ~MegaPoseClient();
  void spin();
};

MegaPoseClient::MegaPoseClient() : Node("MegaPoseClient")
{
  reinitThreshold_ = 0.2;
  initialized_ = false;
  got_image_ = false;
  init_request_done_ = true;
  track_request_done_ = true;
  render_request_done_ = true;
  overlayModel_ = true;
}
MegaPoseClient::~MegaPoseClient()
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shutting down MegaPoseClient");
  rclcpp::shutdown();
}

DetectionMethod MegaPoseClient::getDetectionMethodFromString(const std::string &str)
{
  if (stringToDetectionMethod.find(str) != stringToDetectionMethod.end())
  {
    return stringToDetectionMethod[str];
  }
  return UNKNOWN; // Default case if string is not found
};

void MegaPoseClient::waitForImage()
{
  rclcpp::Rate loop_rate(10);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for a rectified image...");
  while (rclcpp::ok())
  {
    if (got_image_)
    {
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
  rosI_ = image;
  roscam_info_ = cam_info;
  vpI_ = visp_bridge::toVispImageRGBa(*image);
  vpcam_info_ = visp_bridge::toVispCameraParameters(*cam_info);
  width_ = image->width;
  height_ = image->height;

  got_image_ = true;
}
void MegaPoseClient::broadcastTransform(const geometry_msgs::msg::Transform &transform, const std::string &child_frame_id, const std::string &camera_tf)
{
  static auto tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  static geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped.header.stamp = this->get_clock()->now();
  transformStamped.header.frame_id = camera_tf;
  transformStamped.child_frame_id = child_frame_id;
  transformStamped.transform = transform;
  tf_broadcaster_->sendTransform(transformStamped);
}
void MegaPoseClient::spin()
{
  // Get parameters
  std::string image_topic = this->declare_parameter<std::string>("image_topic", "/camera/camera/color/image_raw");
  std::string camera_info_topic = this->declare_parameter<std::string>("camera_info_topic", "/camera/camera/color/camera_info");
  std::string camera_tf = this->declare_parameter<std::string>("camera_tf", "camera_color_optical_frame");
  RCLCPP_INFO(this->get_logger(), "Subscribing to image topic: %s", image_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Subscribing to camera info topic: %s", camera_info_topic.c_str());

  std::string detectorMethod = this->declare_parameter<std::string>("detector_method", "DNN");
  RCLCPP_INFO(this->get_logger(), "Detection method: %s", detectorMethod.c_str());
  if (getDetectionMethodFromString(detectorMethod) == UNKNOWN)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unknown detection method. Exiting.");
    rclcpp::shutdown();
  }

  std::string detectorModelPath = this->declare_parameter<std::string>("detector_model_path", "none");
  if (!fileExists(detectorModelPath))
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Detector model path does not exist. Exiting.");
    rclcpp::shutdown();
  }

  std::string detectorConfig = "none";
  std::string detectorFramework = "onnx", detectorTypeString = "yolov7";
  std::string objectName = this->declare_parameter<std::string>("object_name", "cube");
  RCLCPP_INFO(this->get_logger(), "Object name: %s", objectName.c_str());
  std::vector<std::string> labels = {objectName};

  // Subscribe to image and camera info topics
  raw_image_subscriber.subscribe(this, image_topic, "raw");
  camera_info_subscriber.subscribe(this, camera_info_topic);
  synchronizer_ = std::make_shared<Synchronizer>(SyncPolicy(10), raw_image_subscriber, camera_info_subscriber);
  synchronizer_->registerCallback(
      std::bind(&MegaPoseClient::frameCallback, this, std::placeholders::_1, std::placeholders::_2));
  waitForImage();

  // Initialize DNN detector if detectorMethod is DNN
  if (getDetectionMethodFromString(detectorMethod) == DNN)
  {
    float detectorMeanR = 0.f, detectorMeanG = 0.f, detectorMeanB = 0.f;
    float detectorConfidenceThreshold = 0.65f, detectorNmsThreshold = 0.5f, detectorFilterThreshold = -0.25f;
    float detectorScaleFactor = 0.0039f;
    bool detectorSwapRB = false;

    vpDetectorDNNOpenCV::DNNResultsParsingType detectorType = vpDetectorDNNOpenCV::dnnResultsParsingTypeFromString(detectorTypeString);
    vpDetectorDNNOpenCV::NetConfig netConfig(detectorConfidenceThreshold, detectorNmsThreshold, labels, cv::Size(640, 640), detectorFilterThreshold);
    // vpDetectorDNNOpenCV dnn(netConfig, detectorType);  // I don't know why this doesn't work. If I use this
    // it will cause the error "Cuda and/or GPU driver might not be correctly installed. Setting preferable backend to CPU and trying again.
    // terminate called after throwing an instance of 'cv::Exception"
    dnn_.setNetConfig(netConfig);
    dnn_.setParsingMethod(detectorType);
    dnn_.readNet(detectorModelPath, detectorConfig, detectorFramework);
    dnn_.setMean(detectorMeanR, detectorMeanG, detectorMeanB);
    dnn_.setScaleFactor(detectorScaleFactor);
    dnn_.setSwapRB(detectorSwapRB);
  }

  vpDisplayX *d = NULL;

  d = new vpDisplayX();
  rclcpp::spin_some(this->get_node_base_interface());
  d->init(vpI_); // also init display
  vpDisplay::setTitle(vpI_, "MegaPoseClient debug display");

  auto initial_pose_client = this->create_client<visp_megapose::srv::Init>("initial_pose");

  auto track_pose_client = this->create_client<visp_megapose::srv::Track>("track_pose");

  auto render_client = this->create_client<visp_megapose::srv::Render>("render_object");

  while (!initial_pose_client->wait_for_service(1s) &&
         !track_pose_client->wait_for_service(1s) &&
         !render_client->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "initial_pose service not available, waiting again...");
  }

  rclcpp::Rate loop_rate(10);
  while (rclcpp::ok())
  {
    vpDisplay::display(vpI_);
    rclcpp::spin_some(this->get_node_base_interface());

    if (!initialized_)
    {
      std::optional<vpRect> detection = std::nullopt;
      if (getDetectionMethodFromString(detectorMethod) == DNN)
      {
        detection = detectObjectForInitMegaposeDnn(objectName);
        printf("DNN");
      }

      else if (getDetectionMethodFromString(detectorMethod) == CLICK)
      {
        detection = detectObjectForInitMegaposeClick();
      }

      if (detection && init_request_done_)
      {
        static auto initial_pose_request = std::make_shared<visp_megapose::srv::Init::Request>();
        initial_pose_request->object_name = objectName;
        initial_pose_request->topleft_i = detection->getTopLeft().get_i();
        initial_pose_request->topleft_j = detection->getTopLeft().get_j();
        initial_pose_request->bottomright_i = detection->getBottomRight().get_i();
        initial_pose_request->bottomright_j = detection->getBottomRight().get_j();
        initial_pose_request->image = *rosI_;
        initial_pose_request->camera_info = *roscam_info_;

        auto result_future = initial_pose_client->async_send_request(
            initial_pose_request, std::bind(&MegaPoseClient::initial_pose_service_response_callback, this,
                                            std::placeholders::_1));
        init_request_done_ = false;
      }
    }
    else if (initialized_)
    {
      static auto track_pose_request = std::make_shared<visp_megapose::srv::Track::Request>();
      if (track_request_done_)
      {
        track_pose_request->object_name = objectName;
        track_pose_request->init_pose = transform_;
        track_pose_request->refiner_iterations = 1;
        track_pose_request->image = *rosI_;
        track_pose_request->camera_info = *roscam_info_;

        auto result_future = track_pose_client->async_send_request(
            track_pose_request, std::bind(&MegaPoseClient::track_pose_service_response_callback, this,
                                          std::placeholders::_1));
        track_request_done_ = false;
      }
      static auto render_request = std::make_shared<visp_megapose::srv::Render::Request>();
      if (render_request_done_ && overlayModel_)
      {
        render_request->object_name = objectName;
        render_request->pose = transform_;
        render_request->camera_info = *roscam_info_;
        auto render_result_future = render_client->async_send_request(
            render_request, std::bind(&MegaPoseClient::render_service_response_callback, this, std::placeholders::_1));
        render_request_done_ = false;
      }

      std::string keyboardEvent;
      const bool keyPressed = vpDisplay::getKeyboardEvent(vpI_, keyboardEvent, false);
      if (keyPressed)
      {
        if (keyboardEvent == "t")
          overlayModel_ = !overlayModel_;
      }
      if (overlay_img_.getSize() > 0 && overlayModel_)
        overlayRender(overlay_img_);
      vpDisplay::displayText(vpI_, 20, 20, "Right click to quit", vpColor::red);
      vpDisplay::displayText(vpI_, 30, 20, "Press t: Toggle overlay", vpColor::red);
      static vpHomogeneousMatrix M;
      M = visp_bridge::toVispHomogeneousMatrix(transform_);
      vpDisplay::displayFrame(vpI_, M, vpcam_info_, 0.05, vpColor::none, 3);
      displayScore(confidence_);
      broadcastTransform(transform_, objectName, camera_tf);
    }

    vpDisplay::flush(vpI_);
    vpMouseButton::vpMouseButtonType button;
    if (vpDisplay::getClick(vpI_, button, false))
    {
      if (button == vpMouseButton::button3)
      {
        break; // Right click to stop
      }
    }

    loop_rate.sleep();
  }

  delete d;
}

std::optional<vpRect> MegaPoseClient::detectObjectForInitMegaposeDnn(const std::string &detectionLabel)
{
  cv::Mat I = cv_bridge::toCvCopy(rosI_, rosI_->encoding)->image;
  std::vector<vpDetectorDNNOpenCV::DetectedFeatures2D> detections_vec;
  dnn_.detect(I, detections_vec);
  std::vector<vpDetectorDNNOpenCV::DetectedFeatures2D> matchingDetections;
  for (const auto &detection : detections_vec)
  {
    std::optional<std::string> classnameOpt = detection.getClassName();
    if (classnameOpt)
    {
      if (*classnameOpt == detectionLabel)
      {
        matchingDetections.push_back(detection);
      }
    }
  }
  if (matchingDetections.size() == 0)
  {
    printf("No matching detections\n");
    return std::nullopt;
  }
  else if (matchingDetections.size() == 1)
  {
    printf("One matching detection\n");
    return matchingDetections[0].getBoundingBox();
  }
  else
  {
    printf("Multiple matching detections\n");
    vpRect best;
    double highestConf = 0.0;
    for (const auto &detection : matchingDetections)
    {
      const double conf = detection.getConfidenceScore();
      if (conf > highestConf)
      {
        highestConf = conf;
        best = detection.getBoundingBox();
      }
    }
    return best;
  }

  return std::nullopt;
}
void MegaPoseClient::initial_pose_service_response_callback(rclcpp::Client<visp_megapose::srv::Init>::SharedFuture future)
{
  init_request_done_ = true;
  transform_ = future.get()->pose;
  confidence_ = future.get()->confidence;
  if (confidence_ < reinitThreshold_)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initial pose not reliable, reinitializing...");
  }
  else
  {
    initialized_ = true;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initialized successfully!");
  }
}
void MegaPoseClient::track_pose_service_response_callback(rclcpp::Client<visp_megapose::srv::Track>::SharedFuture future)
{
  track_request_done_ = true;
  transform_ = future.get()->pose;
  confidence_ = future.get()->confidence;
  if (confidence_ < reinitThreshold_)
  {
    initialized_ = false;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Tracking lost, reinitializing...");
  }
}
void MegaPoseClient::render_service_response_callback(rclcpp::Client<visp_megapose::srv::Render>::SharedFuture future)
{
  render_request_done_ = true;
  overlay_img_ = visp_bridge::toVispImageRGBa(future.get()->image);
}
void MegaPoseClient::displayScore(float confidence)
{
  const unsigned top = static_cast<unsigned>(vpI_.getHeight() * 0.85f);
  const unsigned height = static_cast<unsigned>(vpI_.getHeight() * 0.1f);
  const unsigned left = static_cast<unsigned>(vpI_.getWidth() * 0.05f);
  const unsigned width = static_cast<unsigned>(vpI_.getWidth() * 0.5f);
  vpRect full(left, top, width, height);
  vpRect scoreRect(left, top, width * confidence, height);
  const vpColor low = vpColor::red;
  const vpColor high = vpColor::green;
  const vpColor c = interpolate(low, high, confidence);

  vpDisplay::displayRectangle(vpI_, full, c, false, 5);
  vpDisplay::displayRectangle(vpI_, scoreRect, c, true, 1);
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
  const bool startLabelling = vpDisplay::getClick(vpI_, false);

  const vpImagePoint textPosition(10.0, 20.0);

  if (startLabelling)
  {
    vpImagePoint topLeft, bottomRight;
    vpDisplay::displayText(vpI_, textPosition, "Click the upper left corner of the bounding box", vpColor::red);
    vpDisplay::flush(vpI_);
    vpDisplay::getClick(vpI_, topLeft, true);
    vpDisplay::display(vpI_);
    vpDisplay::displayCross(vpI_, topLeft, 5, vpColor::red, 2);
    vpDisplay::displayText(vpI_, textPosition, "Click the bottom right corner of the bounding box", vpColor::red);
    vpDisplay::flush(vpI_);
    vpDisplay::getClick(vpI_, bottomRight, true);
    vpRect bb(topLeft, bottomRight);
    return bb;
  }
  else
  {
    vpDisplay::display(vpI_);
    vpDisplay::displayText(vpI_, textPosition,
                           "Click when the object is visible and static to start reinitializing megapose.",
                           vpColor::red);
    vpDisplay::flush(vpI_);
    return std::nullopt;
  }
}
void MegaPoseClient::overlayRender(const vpImage<vpRGBa> &overlay)
{
  const vpRGBa black = vpRGBa(0, 0, 0);
  for (unsigned int i = 0; i < height_; ++i)
  {
    for (unsigned int j = 0; j < width_; ++j)
    {
      if (overlay[i][j] != black)
      {
        vpI_[i][j] = overlay[i][j];
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