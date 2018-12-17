#ifndef VISP_TRACKER_TRACKER_CLIENT_HH
# define VISP_TRACKER_TRACKER_CLIENT_HH
# include <boost/filesystem/fstream.hpp>
# include <boost/filesystem/path.hpp>
# include <boost/thread/recursive_mutex.hpp>

# include <dynamic_reconfigure/server.h>

# include <image_proc/advertisement_checker.h>

# include <image_transport/image_transport.h>
# include <image_transport/subscriber_filter.h>

# include <message_filters/subscriber.h>
# include <message_filters/sync_policies/approximate_time.h>
# include <message_filters/synchronizer.h>

# include <sensor_msgs/Image.h>
# include <sensor_msgs/CameraInfo.h>

# include <resource_retriever/retriever.h>

# include <visp_tracker/ModelBasedSettingsConfig.h>
# include <visp_tracker/ModelBasedSettingsKltConfig.h>
# include <visp_tracker/ModelBasedSettingsEdgeConfig.h>
# include <visp_tracker/MovingEdgeSites.h>

# include <visp3/core/vpCameraParameters.h>
# include <visp3/core/vpHomogeneousMatrix.h>
# include <visp3/core/vpImage.h>
# include <visp3/mbt/vpMbGenericTracker.h>
# include <visp3/me/vpMe.h>
# include <visp3/klt/vpKltOpencv.h>
# include <visp3/vision/vpPose.h>


namespace visp_tracker
{
  class TrackerClient
  {
  public:
    typedef vpImage<unsigned char> image_t;
    typedef std::vector<vpPoint> points_t;
    typedef std::vector<vpImagePoint> imagePoints_t;

    template<class ConfigType>
    struct reconfigureSrvStruct{
      typedef dynamic_reconfigure::Server<ConfigType> reconfigureSrv_t;
    };

    TrackerClient(ros::NodeHandle& nh,
                  ros::NodeHandle& privateNh,
                  volatile bool& exiting,
                  unsigned queueSize = 5u);
    
    ~TrackerClient();

    void spin();
  protected:
    /// \brief Make sure the topics we subscribe already exist.
    void checkInputs();

    void loadModel();

    bool validatePose(const vpHomogeneousMatrix& cMo);
    vpHomogeneousMatrix loadInitialPose();
    void saveInitialPose(const vpHomogeneousMatrix& cMo);
    points_t loadInitializationPoints();

    void init();
    void initPoint(unsigned& i,
                   points_t& points,
                   imagePoints_t& imagePoints,
                   ros::Rate& rate,
                   vpPose& pose);


    void waitForImage();

    void sendcMo(const vpHomogeneousMatrix& cMo);

    std::string fetchResource(const std::string&);
    bool makeModelFile(boost::filesystem::ofstream& modelStream,
                       const std::string& resourcePath,
                       std::string& fullModelPath);

  private:
    bool exiting ()
    {
      return exiting_ || !ros::ok();
    }

    volatile bool& exiting_;

    unsigned queueSize_;

    ros::NodeHandle& nodeHandle_;
    ros::NodeHandle& nodeHandlePrivate_;

    image_transport::ImageTransport imageTransport_;

    image_t image_;

    std::string modelPath_;
    std::string modelPathAndExt_;
    std::string modelName_;

    std::string cameraPrefix_;
    std::string rectifiedImageTopic_;
    std::string cameraInfoTopic_;
    std::string trackerType_;
    double frameSize_;

    boost::filesystem::path bModelPath_;
    boost::filesystem::path bInitPath_;

    image_transport::CameraSubscriber cameraSubscriber_;

    boost::recursive_mutex mutex_;
    reconfigureSrvStruct<visp_tracker::ModelBasedSettingsConfig>::reconfigureSrv_t *reconfigureSrv_;
    reconfigureSrvStruct<visp_tracker::ModelBasedSettingsKltConfig>::reconfigureSrv_t *reconfigureKltSrv_;
    reconfigureSrvStruct<visp_tracker::ModelBasedSettingsEdgeConfig>::reconfigureSrv_t *reconfigureEdgeSrv_;

    std_msgs::Header header_;
    sensor_msgs::CameraInfoConstPtr info_;

    vpMe movingEdge_;
    vpKltOpencv kltTracker_;
    vpCameraParameters cameraParameters_;
    vpMbGenericTracker tracker_;

    bool startFromSavedPose_;
    bool confirmInit_;

    /// \brief Helper used to check that subscribed topics exist.
    image_proc::AdvertisementChecker checkInputs_;

    resource_retriever::Retriever resourceRetriever_;
  };
} // end of namespace visp_tracker.

#endif //! VISP_TRACKER_TRACKER_CLIENT_HH
