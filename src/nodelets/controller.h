#ifndef __NODELET_CONTROLLER_H__
#define __NODELET_CONTROLLER_H__
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/locks.hpp>
#include <visp/vpDisplay.h>

#include <nodelet/nodelet.h>

#include <visp/vpRGBa.h>
#include "cmd_line/cmd_line.h"

#include "libauto_tracker/tracking.h"
#include "libauto_tracker/tracking_events.h"
#include "tracking_events/display_visp.h"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include <visp_tracker/Init.h>
#include <visp_tracker/UpdatePose.h>
#include "geometry_msgs/TransformStamped.h"
#include <string>


namespace visp_auto_tracker
{

  class AutoTrackerNodelet : public nodelet::Nodelet,public tracking::EventsBase
  {
  public:
    AutoTrackerNodelet ();
    ~AutoTrackerNodelet ();

    void advertiseTopics();
    virtual void onInit ();
    void spinTracker();
    void waitForImage();
    void waitForPattern();

    void on_finished();
    void on_initial_waiting_for_pattern(const vpImage<vpRGBa>& I);
    void on_detect_pattern(const unsigned int iter,const vpImage<vpRGBa>& I, const vpCameraParameters& cam, detectors::DetectorBase& detector);
    void on_redetect_pattern(const unsigned int iter,
                                     const vpImage<vpRGBa>& I,
                                     const  vpCameraParameters& cam,
                                     detectors::DetectorBase& detector,
                                     const vpRect& detection_region);
    void on_detect_model(const vpImage<vpRGBa>& I,
                                 const vpCameraParameters& cam,
                                 const vpHomogeneousMatrix& cMo,
                                 const std::vector<vpImagePoint>& model_inner_corner,
                                 const std::vector<vpImagePoint>& model_outer_corner);
    void on_track_model(const unsigned int iter,
                                const vpImage<vpRGBa>& I,
                                const vpCameraParameters& cam,
                                const vpHomogeneousMatrix& cMo);

    void frameCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cam_info);
    void trackingCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cam_info, const geometry_msgs::TransformStampedConstPtr& transform);
    void trans2matrix(const geometry_msgs::Transform transform, vpHomogeneousMatrix& cMo);

    void runInThread();
  private:
    boost::mutex mutex_tracking_;

    tracking::Tracker* tracker_;
    volatile bool exiting_;
    unsigned int queue_size_;
    bool input_selected_;

    vpDisplay* d_;
    vpImage<vpRGBa> I_;
    vpCameraParameters cam_;
    CmdLine* cmd_;
    vpHomogeneousMatrix cMo_;
    bool cMo_is_initialized;
    unsigned int last_image_seq_;

    ros::Publisher posePublisher_;

    std::string tracker_config_file_;
    int video_reader_width_;
    int video_reader_height_;

    ros::ServiceClient tracker_init_service_;
    visp_tracker::Init tracker_init_comm_;

    ros::ServiceClient tracker_update_pose_service_;
    visp_tracker::UpdatePose tracker_update_pose_comm_;
  };

} // end of namespace visp_tracker.
#endif
