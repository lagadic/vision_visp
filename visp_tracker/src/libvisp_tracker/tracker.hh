#ifndef VISP_TRACKER_TRACKER_HH
# define VISP_TRACKER_TRACKER_HH
# include <boost/filesystem/path.hpp>
# include <boost/thread/recursive_mutex.hpp>

# include <dynamic_reconfigure/server.h>

# include <image_proc/advertisement_checker.h>

# include <image_transport/image_transport.h>

# include <geometry_msgs/TwistStamped.h>

# include <sensor_msgs/Image.h>
# include <sensor_msgs/CameraInfo.h>

# include <tf/transform_broadcaster.h>
# include <tf/transform_listener.h>

# include <visp_tracker/Init.h>
# include <visp_tracker/ModelBasedSettingsConfig.h>
# include <visp_tracker/ModelBasedSettingsKltConfig.h>
# include <visp_tracker/ModelBasedSettingsEdgeConfig.h>
# include <visp_tracker/MovingEdgeSites.h>
# include <visp_tracker/KltPoints.h>

# include <visp3/core/vpCameraParameters.h>
# include <visp3/core/vpHomogeneousMatrix.h>
# include <visp3/core/vpImage.h>
# include <visp3/mbt/vpMbGenericTracker.h>
# include <visp3/me/vpMe.h>

# include <string>

namespace visp_tracker
{
  class Tracker
  {
  public:
    typedef vpImage<unsigned char> image_t;

    typedef boost::function<bool (visp_tracker::Init::Request&,
                                  visp_tracker::Init::Response& res)>
    initCallback_t;

    template<class ConfigType>
    struct reconfigureSrvStruct{
      typedef dynamic_reconfigure::Server<ConfigType> reconfigureSrv_t;
    };

    enum State
    {
      WAITING_FOR_INITIALIZATION,
      TRACKING,
      LOST
    };


    Tracker (ros::NodeHandle& nh,
             ros::NodeHandle& privateNh,
             volatile bool& exiting,
             unsigned queueSize = 5u);
    
    ~Tracker();
    
    void spin();
  protected:
    bool initCallback(visp_tracker::Init::Request& req,
                      visp_tracker::Init::Response& res);

    void updateMovingEdgeSites(visp_tracker::MovingEdgeSitesPtr sites);
    void updateKltPoints(visp_tracker::KltPointsPtr klt);

    void checkInputs();
    void waitForImage();

    void objectPositionHintCallback
    (const geometry_msgs::TransformStampedConstPtr&);
  private:
    bool exiting ()
    {
      return exiting_ || !ros::ok();
    }

    void spinOnce ()
    {
      //callbackQueue_.callAvailable(ros::WallDuration(0));
      ros::spinOnce ();
    }

    volatile bool& exiting_;

    unsigned queueSize_;

    ros::NodeHandle& nodeHandle_;
    ros::NodeHandle& nodeHandlePrivate_;
    image_transport::ImageTransport imageTransport_;

    State state_;
    std::string trackerType_;

    image_t image_;

    std::string cameraPrefix_;
    std::string rectifiedImageTopic_;
    std::string cameraInfoTopic_;

    boost::filesystem::path modelPath_;

    image_transport::CameraSubscriber cameraSubscriber_;

    boost::recursive_mutex mutex_;

    reconfigureSrvStruct<visp_tracker::ModelBasedSettingsConfig>::reconfigureSrv_t *reconfigureSrv_;
    reconfigureSrvStruct<visp_tracker::ModelBasedSettingsKltConfig>::reconfigureSrv_t *reconfigureKltSrv_;
    reconfigureSrvStruct<visp_tracker::ModelBasedSettingsEdgeConfig>::reconfigureSrv_t *reconfigureEdgeSrv_;

    ros::Publisher resultPublisher_;
    ros::Publisher transformationPublisher_;
    tf::TransformBroadcaster tfBroadcaster_;
    ros::Publisher movingEdgeSitesPublisher_;
    ros::Publisher kltPointsPublisher_;

    ros::ServiceServer initService_;
    std_msgs::Header header_;
    sensor_msgs::CameraInfoConstPtr info_;

    vpKltOpencv kltTracker_;
    vpMe movingEdge_;
    vpCameraParameters cameraParameters_;
    vpMbGenericTracker tracker_;

    unsigned lastTrackedImage_;

    /// \brief Helper used to check that subscribed topics exist.
    image_proc::AdvertisementChecker checkInputs_;

    vpHomogeneousMatrix cMo_;

    tf::TransformListener listener_;
    std::string worldFrameId_;
    bool compensateRobotMotion_;

    tf::TransformBroadcaster transformBroadcaster_;
    std::string childFrameId_;

    ros::Subscriber objectPositionHintSubscriber_;
    geometry_msgs::TransformStamped objectPositionHint_;
  };
} // end of namespace visp_tracker.

#endif //! VISP_TRACKER_TRACKER_HH
