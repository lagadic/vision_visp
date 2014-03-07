#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "tracker.hh"

namespace visp_tracker
{
  class TrackerNodelet : public nodelet::Nodelet
  {
  public:
    TrackerNodelet ()
      : nodelet::Nodelet (),
	exiting_ (false),
	tracker_ (),
	thread_ ()
    {}

    ~TrackerNodelet ()
    {
      exiting_ = true;
      if (thread_)
	if (!thread_->timed_join (boost::posix_time::seconds (2)))
	  NODELET_WARN ("failed to join thread but continuing anyway");
      thread_.reset ();
      tracker_.reset ();
    }

    void spin ()
    {
      if (exiting_)
	return;
      tracker_ = boost::shared_ptr<visp_tracker::Tracker>
	(new visp_tracker::Tracker (getMTNodeHandle (),
				    getMTPrivateNodeHandle (),
				    exiting_, 5u));
      while (ros::ok () && !exiting_)
	tracker_->spin ();
    }

    virtual void onInit ()
    {
      NODELET_DEBUG ("Initializing nodelet...");
      exiting_ = false;
      thread_ = boost::make_shared<boost::thread>
	(boost::bind (&TrackerNodelet::spin, this));
    }

  private:
    volatile bool exiting_;
    boost::shared_ptr<visp_tracker::Tracker> tracker_;
    boost::shared_ptr<boost::thread> thread_;
  };

} // end of namespace visp_tracker.

PLUGINLIB_DECLARE_CLASS(visp_tracker, Tracker,
			visp_tracker::TrackerNodelet, nodelet::Nodelet);
