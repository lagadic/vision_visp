#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "tracker-client.hh"

namespace visp_tracker
{
  class TrackerClientNodelet : public nodelet::Nodelet
  {
  public:
    TrackerClientNodelet ()
      : nodelet::Nodelet (),
	exiting_ (false),
	trackerClient_ (),
	thread_ ()
    {}

    ~TrackerClientNodelet ()
    {
      exiting_ = true;
      if (thread_)
	if (!thread_->timed_join (boost::posix_time::seconds (2)))
	  NODELET_WARN ("failed to join thread but continuing anyway");
      thread_.reset ();
      trackerClient_.reset ();
    }

    void spin ()
    {
      trackerClient_ = boost::shared_ptr<visp_tracker::TrackerClient>
	(new visp_tracker::TrackerClient
	 (getMTNodeHandle (),
	  getMTPrivateNodeHandle (),
	  exiting_, 5u));
      if (ros::ok () && !exiting_)
	trackerClient_->spin ();
    }

    virtual void onInit ()
    {
      NODELET_DEBUG ("Initializing nodelet...");
      exiting_ = false;
      thread_ = boost::make_shared<boost::thread>
	(boost::bind (&TrackerClientNodelet::spin, this));
    }
  private:
    volatile bool exiting_;
    boost::shared_ptr<visp_tracker::TrackerClient> trackerClient_;
    boost::shared_ptr<boost::thread> thread_;
  };

} // end of namespace visp_tracker.

PLUGINLIB_DECLARE_CLASS(visp_tracker, TrackerClient,
			visp_tracker::TrackerClientNodelet, nodelet::Nodelet);
