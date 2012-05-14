#ifndef __TRACKING_H__
#define __TRACKING_H__
#include <iostream>
// back-end
#include <boost/msm/back/state_machine.hpp>
//front-end
#include <boost/msm/front/state_machine_def.hpp>
#include <visp/vpImage.h>
#include "cmd_line/cmd_line.h"
#include "datamatrix/detector.h"
#include <visp/vpMbEdgeTracker.h>
#include <visp/vpImage.h>
#include <visp/vpRGBa.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpMbEdgeTracker.h>
#include <visp/vpDisplayX.h>
#include <vector>
#include "states.hpp"
#include <fstream>

namespace msm = boost::msm;
namespace mpl = boost::mpl;
namespace tracking{

  struct input_ready{
    input_ready(vpImage<vpRGBa>& I) : I(I){}
    vpImage<vpRGBa>& I;
  };
  struct select_input{
    select_input(vpImage<vpRGBa>& I) : I(I){}
    vpImage<vpRGBa>& I;
  };


  class Tracker_ : public msm::front::state_machine_def<Tracker_>{
  private:
    CmdLine cmd;
    datamatrix::Detector dmx_detector;
    vpMbEdgeTracker tracker; // Create a model based tracker.
    vpImage<vpRGBa> *I;
    vpImage<vpRGBa> *_I;
    vpHomogeneousMatrix cMo; // Pose computed using the tracker.
    vpCameraParameters cam;
    vpImage<unsigned char> Igray;

    std::vector<vpPoint> points3D_inner;
    std::vector<vpPoint> points3D_outer;
    std::vector<vpPoint> f;

    std::ofstream varfile_;
    int iter_;
  public:
    datamatrix::Detector& get_dmx_detector();
    vpMbEdgeTracker& get_mbt();
    std::vector<vpPoint>& get_points3D_inner();
    std::vector<vpPoint>& get_points3D_outer();
    std::vector<vpPoint>& get_flashcode();
    vpImage<vpRGBa>& get_I();
    vpCameraParameters& get_cam();

    Tracker_(CmdLine& cmd);

    typedef WaitingForInput initial_state;
    bool input_selected(input_ready const& evt);
    bool no_input_selected(input_ready const& evt);
    bool flashcode_detected(input_ready const& evt);
    bool model_detected(msm::front::none const&);
    bool mbt_success(input_ready const& evt);

    void find_flashcode_pos(input_ready const& evt);
    void track_model(input_ready const& evt);

    void track(vpImage<vpRGBa>& I);

    struct transition_table : mpl::vector<
      //    Start               Event              Target                  Action                         Guard
      //   +-----------------+--------------------+-----------------------+------------------------------+------------------------------+
      g_row< WaitingForInput , input_ready        , WaitingForInput       ,                               &Tracker_::no_input_selected    >,
      //   +-----------------+--------------------+-----------------------+------------------------------+------------------------------+
      g_row< WaitingForInput , input_ready        , DetectFlashcode       ,                               &Tracker_::input_selected       >,
      //   +-----------------+--------------------+-----------------------+------------------------------+------------------------------+
       _row< WaitingForInput , select_input       , DetectFlashcode                                                                       >,
      //   +-----------------+--------------------+-----------------------+------------------------------+------------------------------+
       _row< DetectFlashcode , input_ready        , DetectFlashcode                                        /* default behaviour */        >,
      //   +-----------------+--------------------+-----------------------+------------------------------+------------------------------+
        row< DetectFlashcode , input_ready        , DetectModel           , &Tracker_::find_flashcode_pos,&Tracker_::flashcode_detected   >,
      //   +-----------------+--------------------+-----------------------+------------------------------+------------------------------+
      g_row< DetectModel     , msm::front::none   , TrackModel            ,                               &Tracker_::model_detected       >,
      //   +-----------------+--------------------+-----------------------+------------------------------+------------------------------+
       _row< TrackModel      , input_ready        , DetectFlashcode                                        /* default behaviour */        >,
      //   +-----------------+--------------------+-----------------------+------------------------------+------------------------------+
        row< TrackModel      , input_ready        , TrackModel           , &Tracker_::track_model        ,&Tracker_::mbt_success          >
      //   +-----------------+--------------------+-----------------------+------------------------------+------------------------------+

      > {};

  };

  typedef msm::back::state_machine<Tracker_> Tracker;


}
#endif //__TRACKING_H__
