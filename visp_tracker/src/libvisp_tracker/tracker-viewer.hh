#ifndef VISP_TRACKER_TRACKER_VIEWER_HH
# define VISP_TRACKER_TRACKER_VIEWER_HH
# include <boost/filesystem/path.hpp>
# include <boost/optional.hpp>

# include <geometry_msgs/PoseWithCovarianceStamped.h>

# include <image_proc/advertisement_checker.h>

# include <image_transport/image_transport.h>
# include <image_transport/subscriber_filter.h>

# include <message_filters/subscriber.h>
# include <message_filters/sync_policies/approximate_time.h>
# include <message_filters/synchronizer.h>

# include <sensor_msgs/Image.h>
# include <sensor_msgs/CameraInfo.h>

# include <visp_tracker/Init.h>
# include <visp_tracker/MovingEdgeSites.h>
# include <visp_tracker/KltPoints.h>

# include <visp/vpCameraParameters.h>
# include <visp/vpImage.h>
# include <visp/vpMbEdgeTracker.h>

namespace visp_tracker
{
  /// \brief Monitors the tracking result provided by the tracking node.
  class TrackerViewer
  {
  public:
    /// \brief ViSP image type
    typedef vpImage<unsigned char> image_t;


    typedef boost::function<bool (visp_tracker::Init::Request&,
          visp_tracker::Init::Response& res)>
    initCallback_t;

    typedef boost::function<bool (visp_tracker::Init::Request&,
          visp_tracker::Init::Response& res)>
    reconfigureCallback_t;

    /// \brief Synchronization policy
    ///
    /// This is used to make sure that the image, the object position
    /// and the moving edge sites are synchronized. This may not be the
    /// case as these informations are published on different topics.
    /// The approximate time allows light differences in timestamps
    /// which are not critical as this is only a viewer.
    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image, sensor_msgs::CameraInfo,
      geometry_msgs::PoseWithCovarianceStamped,
      visp_tracker::MovingEdgeSites,
      visp_tracker::KltPoints
      > syncPolicy_t;

    /// \brief Constructor.
    TrackerViewer(ros::NodeHandle& nh,
		  ros::NodeHandle& privateNh,
		  volatile bool& exiting,
		  unsigned queueSize = 5u);

    /// \brief Display camera image, tracked object position and moving
    /// edge sites.
    void spin();
  protected:
    /// \brief Initialize the tracker.
    void initializeTracker();

    /// \brief Initialize the common parameters (visibility angles, etc)
    void loadCommonParameters();

    /// \brief Make sure the topics we subscribe already exist.
    void checkInputs();

    /// \brief Hang until the first image is received.
    void waitForImage();

    bool initCallback(visp_tracker::Init::Request& req,
          visp_tracker::Init::Response& res);

    bool reconfigureCallback(visp_tracker::Init::Request& req,
          visp_tracker::Init::Response& res);

    /// \brief Callback used to received synchronized data.
    void
    callback
    (const sensor_msgs::ImageConstPtr& imageConst,
     const sensor_msgs::CameraInfoConstPtr& infoConst,
     const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& trackingResult,
     const visp_tracker::MovingEdgeSites::ConstPtr& sitesConst,
     const visp_tracker::KltPoints::ConstPtr& kltConst);

    void timerCallback();

    /// \brief Display moving edge sites.
    void displayMovingEdgeSites();
    /// \brief Display KLT points that are tracked.
    void displayKltPoints();

  private:
    bool exiting ()
    {
      return exiting_ || !ros::ok();
    }

    volatile bool& exiting_;

    /// \brief Queue size for all subscribers.
    unsigned queueSize_;

    ros::NodeHandle& nodeHandle_;
    ros::NodeHandle& nodeHandlePrivate_;

    /// \brief Image transport used to receive images.
    image_transport::ImageTransport imageTransport_;

    double frameSize_;

    /// \name Topics and services strings.
    /// \{

    /// \brief Full topic name for rectified image.
    std::string rectifiedImageTopic_;
    /// \brief Full topic name for camera information.
    std::string cameraInfoTopic_;

    /// \}

    /// \brief Service called when user ends tracker_client node
    ros::ServiceServer initService_;

    /// \brief Service called when user is reconfiguring tracker node
    ros::ServiceServer reconfigureService_;

    /// \brief Name of the tracker used in this viewer node
    std::string trackerName_;

    /// \brief Model path.
    boost::filesystem::path modelPath_;

    /// \brief Helper used to check that subscribed topics exist.
    image_proc::AdvertisementChecker checkInputs_;

    /// \brief ViSP edge tracker.
    vpMbEdgeTracker tracker_;
    /// \brief ViSP camera parameters.
    vpCameraParameters cameraParameters_;
    /// \brief ViSP image.
    image_t image_;

    /// \brief Shared pointer to latest received camera information.
    sensor_msgs::CameraInfoConstPtr info_;
    /// \brief Last tracked object position, set to none if tracking failed.
    boost::optional<vpHomogeneousMatrix> cMo_;
    /// \brief Shared pointer to latest received moving edge sites.
    visp_tracker::MovingEdgeSites::ConstPtr sites_;
    /// \brief Shared pointer to latest received KLT point positions.
    visp_tracker::KltPoints::ConstPtr klt_;

    /// \name Subscribers and synchronizer.
    /// \{
    /// \brief Subscriber to image topic.
    image_transport::SubscriberFilter imageSubscriber_;
    /// \brief Subscriber to camera information topic.
    message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoSubscriber_;
    /// \brief Subscriber to tracking result topic.
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>
    trackingResultSubscriber_;
    /// \brief Subscriber to moving edge sites topics.
    message_filters::Subscriber<visp_tracker::MovingEdgeSites>
    movingEdgeSitesSubscriber_;
    /// \brief Subscriber to KLT point topics.
    message_filters::Subscriber<visp_tracker::KltPoints>
    kltPointsSubscriber_;

    /// \brief Synchronizer with approximate time policy.
    message_filters::Synchronizer<syncPolicy_t> synchronizer_;
    ///}

    /// \name Synchronization check
    /// \{
    ros::WallTimer timer_;
    unsigned countAll_;
    unsigned countImages_;
    unsigned countCameraInfo_;
    unsigned countTrackingResult_;
    unsigned countMovingEdgeSites_;
    unsigned countKltPoints_;
    ///}
  };
} // end of namespace visp_tracker

#endif //! VISP_TRACKER_TRACKER_VIEWER_HH
