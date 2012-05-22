#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "tracker.hh"

namespace visp_tracker
{
  class TrackerNodelet : public nodelet::Nodelet
  {
  public:
    virtual void onInit ()
    {
      NODELET_DEBUG ("Initializing nodelet...");
      tracker_ = boost::make_shared<visp_tracker::Tracker> ();
    }
  private:
    boost::shared_ptr<visp_tracker::Tracker> tracker_;
  };

} // end of namespace visp_tracker.

PLUGINLIB_DECLARE_CLASS(visp_tracker, Tracker,
			visp_tracker::TrackerNodelet, nodelet::Nodelet);
