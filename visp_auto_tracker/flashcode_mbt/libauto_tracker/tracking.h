#ifndef __TRACKING_H__
#define __TRACKING_H__
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/accumulators/statistics/median.hpp>
#include <boost/accumulators/statistics/p_square_quantile.hpp>

#include <iostream>
// back-end
#include <boost/msm/back/state_machine.hpp>
//front-end
#include <boost/msm/front/state_machine_def.hpp>
#include <boost/array.hpp>

#include <visp3/core/vpImage.h>
#include <visp3/core/vpRGBa.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpCameraParameters.h>

#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpHinkley.h>
#include <visp3/me/vpMe.h>
#include <vector>
#include <fstream>

#include "visp_tracker/MovingEdgeSites.h"
#include "visp_tracker/KltPoints.h"

#include "cmd_line/cmd_line.h"
#if VISP_VERSION_INT < VP_VERSION_INT(2,10,0)
#include "detectors/detector_base.h"
#else
#  include <visp/vpDetectorBase.h>
#endif

#include <visp/vpMbEdgeTracker.h>
#include "states.hpp"
#include "events.h"

using namespace boost::accumulators;
namespace msm = boost::msm;
namespace mpl = boost::mpl;
namespace tracking{

  class Tracker_ : public msm::front::state_machine_def<Tracker_>{
  public:
    typedef struct {
      boost::accumulators::accumulator_set<
      double,
      boost::accumulators::stats<
      boost::accumulators::tag::median(boost::accumulators::with_p_square_quantile),
      boost::accumulators::tag::max,
      boost::accumulators::tag::mean
      >
      > var,var_x,var_y,var_z,var_wx,var_wy,var_wz,checkpoints;

    } statistics_t;
  private:
    CmdLine cmd;
    int iter_;
    vpImagePoint flashcode_center_;
    std::ofstream varfile_;
#if VISP_VERSION_INT < VP_VERSION_INT(2,10,0)
    detectors::DetectorBase* detector_;
#else
    vpDetectorBase *detector_;
#endif
    typedef boost::array<vpHinkley,6> hinkley_array_t;
    hinkley_array_t hink_;

    vpMbTracker* tracker_; // Create a model based tracker.
    vpMe tracker_me_config_;
    vpImage<vpRGBa> *I_;
    vpImage<vpRGBa> *_I;
    vpHomogeneousMatrix cMo_; // Pose computed using the tracker.
    vpMatrix covariance_; // Covariance associated to the pose estimation
    vpCameraParameters cam_;
    vpImage<unsigned char> Igray_;

    std::vector<vpPoint> outer_points_3D_bcp_;
    std::vector<vpPoint> points3D_inner_;
    std::vector<vpPoint> points3D_outer_;
    std::vector<vpPoint> points3D_middle_;
    std::vector<vpPoint> f_;
    vpRect vpTrackingBox_;
    cv::Rect cvTrackingBox_;
    bool cvTrackingBox_init_;

    statistics_t statistics;
    bool flush_display_;

  public:
    //getters to access useful members
    void set_flush_display(bool val);
    bool get_flush_display();
#if VISP_VERSION_INT < VP_VERSION_INT(2,10,0)
    detectors::DetectorBase& get_detector();
#else
    vpDetectorBase& get_detector();
#endif
    vpMbTracker& get_mbt();
    std::vector<vpPoint>& get_points3D_inner();
    std::vector<vpPoint>& get_points3D_outer();
    std::vector<vpPoint>& get_points3D_middle();
    std::vector<vpPoint>& get_flashcode();

    //returns tracking box where to look for pattern (may be full image)
    template<class T>
    const T& get_tracking_box();
    //returns currently treated image
    vpImage<vpRGBa>& get_I();
    //returns camera parameters
    vpCameraParameters& get_cam();
    //returns tracker configuration
    CmdLine& get_cmd();

    //constructor
    //inits tracker from a detector, a visp tracker

    Tracker_(CmdLine& cmd, vpDetectorBase* detector, vpMbTracker *tracker_, bool flush_display = true);

    typedef WaitingForInput initial_state;      //initial state of our state machine tracker

    //Guards
    bool input_selected(input_ready const& evt);
    bool no_input_selected(input_ready const& evt);
    bool flashcode_detected(input_ready const& evt);
    bool flashcode_redetected(input_ready const& evt);
    bool model_detected(msm::front::none const&);
    bool mbt_success(input_ready const& evt);

    //actions
    void find_flashcode_pos(input_ready const& evt);
    void track_model(input_ready const& evt);

    //gets statistics about the last tracking experience
    statistics_t& get_statistics();

    void updateMovingEdgeSites(visp_tracker::MovingEdgeSitesPtr sites);
    void updateKltPoints(visp_tracker::KltPointsPtr klt);

    //here is how the tracker works
    struct transition_table : mpl::vector<
        //    Start               Event              Target                       Action                         Guard
        //   +------------------+--------------------+-----------------------+------------------------------+------------------------------+
        g_row< WaitingForInput  , input_ready        , WaitingForInput       ,                               &Tracker_::no_input_selected    >,
        //   +------------------+--------------------+-----------------------+------------------------------+------------------------------+
        g_row< WaitingForInput  , input_ready        , DetectFlashcode       ,                               &Tracker_::input_selected       >,
        //   +------------------+--------------------+-----------------------+------------------------------+------------------------------+
        _row< WaitingForInput  , select_input       , DetectFlashcode                                                                       >,
        //   +------------------+--------------------+-----------------------+------------------------------+------------------------------+
        _row< DetectFlashcode  , input_ready        , DetectFlashcode                                        /* default behaviour */        >,
        //   +------------------+--------------------+-----------------------+------------------------------+------------------------------+
        row< DetectFlashcode  , input_ready        , DetectModel           , &Tracker_::find_flashcode_pos,&Tracker_::flashcode_detected   >,
        //   +------------------+--------------------+-----------------------+------------------------------+------------------------------+
        _row< DetectModel      , msm::front::none   , DetectFlashcode                                          /* default behaviour */      >,
        //   +------------------+--------------------+-----------------------+------------------------------+------------------------------+
        g_row< DetectModel      , msm::front::none   , TrackModel            ,                               &Tracker_::model_detected       >,
        //   +------------------+--------------------+-----------------------+------------------------------+------------------------------+
        _row< TrackModel       , input_ready        , ReDetectFlashcode                                        /* default behaviour */      >,
        //   +------------------+--------------------+-----------------------+------------------------------+------------------------------+
        row< TrackModel       , input_ready        , TrackModel            , &Tracker_::track_model       ,&Tracker_::mbt_success          >,
        //   +------------------+--------------------+-----------------------+------------------------------+------------------------------+
        _row< ReDetectFlashcode, input_ready        , DetectFlashcode                                        /* default behaviour */        >,
        //   +------------------+--------------------+-----------------------+------------------------------+------------------------------+
        row< ReDetectFlashcode, input_ready        , DetectModel           , &Tracker_::find_flashcode_pos,&Tracker_::flashcode_redetected >,
        //   +------------------+--------------------+-----------------------+------------------------------+------------------------------+
        //row< ReDetectFlashcode, input_ready        , TrackModel            , &Tracker_::track_model       ,&Tracker_::mbt_success          >,
        //   +------------------+--------------------+-----------------------+------------------------------+------------------------------+
        _row< TrackModel       , finished           , Finished                                                                              >,
        //   +------------------+--------------------+-----------------------+------------------------------+------------------------------+
        _row< DetectModel      , finished           , Finished                                                                              >,
        //   +------------------+--------------------+-----------------------+------------------------------+------------------------------+
        _row< DetectFlashcode  , finished           , Finished                                                                              >,
        //   +------------------+--------------------+-----------------------+------------------------------+------------------------------+
        _row< ReDetectFlashcode, finished           , Finished                                                                              >
        //   +------------------+--------------------+-----------------------+------------------------------+------------------------------+
        > {};

  };

  typedef msm::back::state_machine<Tracker_> Tracker;


}
#endif //__TRACKING_H__
