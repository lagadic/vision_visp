#ifndef __NODELET_CONTROLLER_H__
#define __NODELET_CONTROLLER_H__
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>


namespace visp_auto_tracker
{
  class AutoTrackerNodelet : public nodelet::Nodelet
  {
  public:
    AutoTrackerNodelet ();
    ~AutoTrackerNodelet ();

    virtual void onInit ();
  private:
    volatile bool exiting_;
    //boost::shared_ptr<visp_tracker::Tracker> tracker_;
    //boost::shared_ptr<boost::thread> thread_;
  };

} // end of namespace visp_tracker.
#endif
