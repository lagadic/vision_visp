//command line parameters
#include "cmd_line/cmd_line.h"

//detectors
#include "detectors/datamatrix/detector.h"
#include "detectors/qrcode/detector.h"

//tracking
#include "libauto_tracker/tracking.h"
#include "libauto_tracker/threading.h"
#include "libauto_tracker/events.h"

//visp includes
#include <visp/vpImageIo.h>
#include <visp/vpVideoReader.h>
#include <visp/vpMbEdgeKltTracker.h>
#include <visp/vpMbKltTracker.h>
#include <visp/vpMbEdgeTracker.h>
#include <visp/vpDisplayX.h>

int main(int argc, char**argv)
{
  //Parse command line arguments
  CmdLine cmd(argc,argv);

  if(cmd.should_exit()) return 0; //exit if needed

  //Read video from a set of images, a single image or a camera
  vpImage<vpRGBa> I;
  vpVideoReader reader;
  vpMbTracker* tracker;

  vpCameraParameters cam = cmd.get_cam_calib_params();
  if(cmd.get_verbose())
    std::cout << "loaded camera parameters:" << cam << std::endl;

  std::string filenames((cmd.get_data_dir() + cmd.get_input_file_pattern()));
  if(cmd.get_verbose())
    std::cout << "Loading: " << filenames << std::endl;
  reader.setFileName( filenames.c_str() );

  reader.setFirstFrameIndex(2);
  reader.open(I);

  //init display
  vpDisplayX* d = new vpDisplayX();
  d->init(I);
  //init hybrid tracker
  detectors::DetectorBase* detector = NULL;
  if (cmd.get_detector_type() == CmdLine::ZBAR)
    detector = new detectors::qrcode::Detector;
  else if(cmd.get_detector_type() == CmdLine::DTMX)
    detector = new detectors::datamatrix::Detector;

  if(cmd.get_tracker_type() == CmdLine::KLT)
    tracker = new vpMbKltTracker();
  else if(cmd.get_tracker_type() == CmdLine::KLT_MBT)
    tracker = new vpMbEdgeKltTracker();
  else if(cmd.get_tracker_type() == CmdLine::MBT)
    tracker = new vpMbEdgeTracker();

  tracking::Tracker t(cmd,detector,tracker);

  t.start();
  reader.acquire(I);

  t.process_event(tracking::select_input(I));
  for(int iter=0;(iter<reader.getLastFrameIndex()-1); iter++){
    reader.acquire(I);
    t.process_event(tracking::input_ready(I,cam,iter));
  }

  t.process_event(tracking::finished());
}
