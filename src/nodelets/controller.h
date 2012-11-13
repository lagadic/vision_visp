#ifndef __NODELET_CONTROLLER_H__
#define __NODELET_CONTROLLER_H__
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <visp/vpDisplay.h>

#include <nodelet/nodelet.h>

#include <visp/vpRGBa.h>
#include "cmd_line/cmd_line.h"

#include "libauto_tracker/tracking.h"
#include "libauto_tracker/tracking_events.h"
#include "tracking_events/display_visp.h"

#include <string>


namespace visp_auto_tracker
{

  class AutoTrackerNodelet : public nodelet::Nodelet,public tracking_events::DisplayVispEvents
  {
  public:
    AutoTrackerNodelet ();
    ~AutoTrackerNodelet ();

    void advertiseTopics();
    virtual void onInit ();
    void spinTracker();
    void spinMachine();

    void on_finished();
  private:
    volatile bool exiting_;
    boost::shared_ptr<tracking::Tracker> tracker_;
    boost::shared_ptr<boost::thread> thread_;
    ros::Publisher posePublisher_;
    vpDisplay* d_;
    vpImage<vpRGBa> I_;
    CmdLine* cmd_;

    std::string tracker_config_file_;
    int video_reader_width_;
    int video_reader_height_;
  };

} // end of namespace visp_tracker.
#endif
