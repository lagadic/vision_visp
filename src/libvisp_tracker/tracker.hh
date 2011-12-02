#ifndef VISP_TRACKER_TRACKER_HH
# define VISP_TRACKER_TRACKER_HH
# include <boost/circular_buffer.hpp>
# include <boost/filesystem/path.hpp>

# include <dynamic_reconfigure/server.h>

# include <image_proc/advertisement_checker.h>

# include <image_transport/image_transport.h>

# include <geometry_msgs/TwistStamped.h>

# include <sensor_msgs/Image.h>
# include <sensor_msgs/CameraInfo.h>

# include <tf/transform_broadcaster.h>

# include <visp_tracker/Init.h>
# include <visp_tracker/MovingEdgeConfig.h>
# include <visp_tracker/MovingEdgeSites.h>
# include <visp_tracker/TrackingResult.h>

# include <visp/vpCameraParameters.h>
# include <visp/vpHomogeneousMatrix.h>
# include <visp/vpImage.h>
# include <visp/vpMbEdgeTracker.h>

namespace visp_tracker
{
  class Tracker
  {
  public:
    typedef vpImage<unsigned char> image_t;

    typedef dynamic_reconfigure::Server<visp_tracker::MovingEdgeConfig>
    reconfigureSrv_t;

    typedef boost::function<bool (visp_tracker::Init::Request&,
				  visp_tracker::Init::Response& res)>
    initCallback_t;

    enum State
      {
	WAITING_FOR_INITIALIZATION,
	TRACKING,
	LOST
      };


    Tracker(unsigned queueSize = 5u);
    void spin();
  protected:
    bool initCallback(visp_tracker::Init::Request& req,
		      visp_tracker::Init::Response& res);
    void
    cameraVelocityCallback(const geometry_msgs::TwistStampedConstPtr& twist);

    void updateMovingEdgeSites();

    void checkInputs();
    void waitForImage();

    void integrateCameraVelocity(const std_msgs::Header& lastHeader,
				 const std_msgs::Header& currentHeader);

    std::string velocitiesDebugMessage();
  private:
    typedef std::pair<double, vpColVector> velocityDuringInterval_t;
    typedef boost::circular_buffer<velocityDuringInterval_t> velocities_t;
    static const velocities_t::size_type MAX_VELOCITY_VALUES = 1000;

    unsigned queueSize_;

    ros::NodeHandle nodeHandle_;
    image_transport::ImageTransport imageTransport_;

    State state_;

    image_t image_;

    std::string cameraPrefix_;
    std::string rectifiedImageTopic_;
    std::string cameraInfoTopic_;

    boost::filesystem::path vrmlPath_;

    image_transport::CameraSubscriber cameraSubscriber_;

    reconfigureSrv_t reconfigureSrv_;
    ros::Publisher resultPublisher_;
    ros::Publisher transformationPublisher_;
    tf::TransformBroadcaster tfBroadcaster_;
    ros::Publisher movingEdgeSitesPublisher_;
    ros::Subscriber cameraVelocitySubscriber_;

    ros::ServiceServer initService_;

    std_msgs::Header header_;
    sensor_msgs::CameraInfoConstPtr info_;

    vpMe movingEdge_;
    vpCameraParameters cameraParameters_;
    vpMbEdgeTracker tracker_;

    visp_tracker::MovingEdgeSites sites_;

    unsigned lastTrackedImage_;

    /// \brief Helper used to check that subscribed topics exist.
    image_proc::AdvertisementChecker checkInputs_;

    vpHomogeneousMatrix cMo_;

    velocities_t velocities_;

    tf::TransformBroadcaster transformBroadcaster_;
    std::string childFrameId_;
  };
} // end of namespace visp_tracker.

#endif //! VISP_TRACKER_TRACKER_HH
