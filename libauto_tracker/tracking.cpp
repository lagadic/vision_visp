#include "cv.h"
#include "highgui.h"
#include "tracking.h"
#include <visp/vpImageConvert.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpImagePoint.h>
#include <visp/vpDisplayX.h>
#include <visp/vpPose.h>
#include <visp/vpMeterPixelConversion.h>
#include <visp/vpTrackingException.h>
#include <visp/vpImageIo.h>
#include <visp/vpRect.h>
#include "logfilewriter.hpp"

namespace tracking{

  Tracker_:: Tracker_(CmdLine& cmd, detectors::DetectorBase* detector,vpMbTracker* tracker,bool flush_display) :
      cmd(cmd),
      iter_(0),
      flashcode_center_(640/2,480/2),
      detector_(detector),
      tracker_(tracker),
      flush_display_(flush_display){
    std::cout << "starting tracker" << std::endl;
    points3D_inner_ = cmd.get_inner_points_3D();
    points3D_outer_ = cmd.get_outer_points_3D();
    outer_points_3D_bcp_ = cmd.get_outer_points_3D();
    if(cmd.using_adhoc_recovery() || cmd.log_checkpoints()){
      for(int i=0;i<points3D_outer_.size();i++){
        vpPoint p;
        p.setWorldCoordinates(
                  (points3D_outer_[i].get_oX()+points3D_inner_[i].get_oX())*cmd.get_adhoc_recovery_ratio(),
                  (points3D_outer_[i].get_oY()+points3D_inner_[i].get_oY())*cmd.get_adhoc_recovery_ratio(),
                  (points3D_outer_[i].get_oZ()+points3D_inner_[i].get_oZ())*cmd.get_adhoc_recovery_ratio()
                );
        points3D_middle_.push_back(p);
      }
    }
    f_ = cmd.get_flashcode_points_3D();

    if(cmd.using_var_file()){
      varfile_.open(cmd.get_var_file().c_str(),std::ios::out);
      varfile_ << "#These are variances from the model based tracker in gnuplot format" << std::endl;
      if(cmd.using_hinkley())
        varfile_ << "iteration\tvar_x\var_y\tvar_z\tvar_wx\tvar_wy\var_wz";
      if(cmd.using_mbt_dynamic_range())
        varfile_ << "\tmbt_range";
      if(cmd.log_pose())
        varfile_ << "\tpose_tx\tpose_ty\tpose_tz\tpose_rx\tpose_ry\tpose_rz";
      if(cmd.log_checkpoints())
        varfile_ << "\tcheckpoint_variance";

      varfile_ << std::endl;
    }

    if(cmd.using_hinkley()){
      if(cmd.get_verbose())
        std::cout << "Initialising hinkley with alpha=" << cmd.get_hinkley_alpha() << " and delta=" << cmd.get_hinkley_delta() << std::endl;
      for(hinkley_array_t::iterator i = hink_.begin();i!=hink_.end();i++)
        i->init(cmd.get_hinkley_alpha(),cmd.get_hinkley_delta());
    }

    if(cmd.using_mbt_dynamic_range()){
      vpMbEdgeTracker *tracker_me = dynamic_cast<vpMbEdgeTracker*>(tracker_);
      if(tracker_me)
        tracker_me->getMovingEdge(tracker_me_config_);
      else
        std::cout << "error: could not init moving edges on tracker that doesn't support them." << std::endl;
    }

    tracker_->loadConfigFile(cmd.get_xml_file().c_str() ); // Load the configuration of the tracker
    tracker_->loadModel(cmd.get_wrl_file().c_str()); // load the 3d model, to read .wrl model the 3d party library coin is required, if coin is not installed .cao file can be used.
  }

  detectors::DetectorBase& Tracker_:: get_detector(){
    return *detector_;
  }

  vpMbTracker& Tracker_:: get_mbt(){
    return *tracker_;
  }

  std::vector<vpPoint>& Tracker_:: get_points3D_inner(){
    return points3D_inner_;
  }

  std::vector<vpPoint>& Tracker_:: get_points3D_outer(){
    return points3D_outer_;

  }

  std::vector<vpPoint>& Tracker_:: get_points3D_middle(){
    return points3D_middle_;
  }

  std::vector<vpPoint>& Tracker_:: get_flashcode(){
    return f_;
  }

  vpImage<vpRGBa>& Tracker_:: get_I(){
    return *I_;
  }

  vpCameraParameters& Tracker_:: get_cam(){
    return cam_;
  }

  CmdLine& Tracker_:: get_cmd(){
    return cmd;
  }

  template<>
  const cv::Rect& Tracker_:: get_tracking_box<cv::Rect>(){
    return cvTrackingBox_;
  }

  template<>
  const vpRect& Tracker_:: get_tracking_box<vpRect>(){
    return vpTrackingBox_;
  }

  bool Tracker_:: input_selected(input_ready const& evt){
    return vpDisplay::getClick(evt.I,false);
  }


  bool Tracker_:: no_input_selected(input_ready const& evt){
    return !input_selected(evt);
  }

  bool Tracker_:: flashcode_detected(input_ready const& evt){
    this->cam_ = evt.cam_;
    clock_t t = clock();
    cv::Mat cvI;//(evt.I.getRows(),evt.I.getCols(),CV_8UC3);

    cv::Mat vpToMat((int)evt.I.getRows(), (int)evt.I.getCols(), CV_8UC4, (void*)evt.I.bitmap);

    cvI = cv::Mat((int)evt.I.getRows(), (int)evt.I.getCols(), CV_8UC3);
    cv::Mat alpha((int)evt.I.getRows(), (int)evt.I.getCols(), CV_8UC1);

    cv::Mat out[] = {cvI, alpha};
    int from_to[] = { 0,2,  1,1,  2,0,  3,3 };
    cv::mixChannels(&vpToMat, 1, out, 2, from_to, 4);

    //vpImageConvert::convert(evt.I,cvI);



    return detector_->detect(cvI,cmd.get_dmx_timeout(),0,0);
  }

  /*
   * Detect flashcode in region delimited by the outer points of the model
   * The timeout is the default timeout times the surface ratio
   */
  bool Tracker_:: flashcode_redetected(input_ready const& evt){
    this->cam_ = evt.cam_;
    clock_t t = clock();
    cv::Mat cvI;

    //vpImageConvert::convert(evt.I,cvI);
    cv::Mat vpToMat((int)evt.I.getRows(), (int)evt.I.getCols(), CV_8UC4, (void*)evt.I.bitmap);

    cvI = cv::Mat((int)evt.I.getRows(), (int)evt.I.getCols(), CV_8UC3);
    cv::Mat alpha((int)evt.I.getRows(), (int)evt.I.getCols(), CV_8UC1);

    cv::Mat out[] = {cvI, alpha};
    int from_to[] = { 0,2,  1,1,  2,0,  3,3 };
    cv::mixChannels(&vpToMat, 1, out, 2, from_to, 4);
    cv::Mat subImage = cv::Mat(cvI,get_tracking_box<cv::Rect>()).clone();

    double timeout = cmd.get_dmx_timeout()*(double)(get_tracking_box<cv::Rect>().width*get_tracking_box<cv::Rect>().height)/(double)(cvI.cols*cvI.rows);
    return detector_->detect(subImage,(unsigned int)timeout,get_tracking_box<cv::Rect>().x,get_tracking_box<cv::Rect>().y);
  }

  void Tracker_:: find_flashcode_pos(input_ready const& evt){
    std::vector<cv::Point> polygon = detector_->get_polygon();
    double centerX = (double)(polygon[0].x+polygon[1].x+polygon[2].x+polygon[3].x)/4.;
    double centerY = (double)(polygon[0].y+polygon[1].y+polygon[2].y+polygon[3].y)/4.;
    vpPixelMeterConversion::convertPoint(cam_, flashcode_center_, centerX, centerY);

    for(int i=0;i<f_.size();i++){
      double x=0, y=0;
      vpImagePoint poly_pt(polygon[i].y,polygon[i].x);

      vpPixelMeterConversion::convertPoint(cam_, poly_pt, x, y);
      f_[i].set_x(x);
      f_[i].set_y(y);
    }
    I_ = _I = &(evt.I);
  }


  bool Tracker_:: model_detected(msm::front::none const&){
    vpImageConvert::convert(*I_,Igray_);
    vpPose pose;

    for(int i=0;i<f_.size();i++)
      pose.addPoint(f_[i]);

    pose.computePose(vpPose::LAGRANGE,cMo_);
    pose.computePose(vpPose::VIRTUAL_VS,cMo_);
    vpDisplay::displayFrame(*I_,cMo_,cam_,0.01,vpColor::none,2);

    std::vector<vpImagePoint> model_inner_corner(4);
    std::vector<vpImagePoint> model_outer_corner(4);
    for(int i=0;i<4;i++){
      points3D_outer_[i].project(cMo_);
      points3D_inner_[i].project(cMo_);
      if(cmd.using_adhoc_recovery() || cmd.log_checkpoints())
        points3D_middle_[i].project(cMo_);
      vpMeterPixelConversion::convertPoint(cam_,points3D_outer_[i].get_x(),points3D_outer_[i].get_y(),model_outer_corner[i]);
      vpMeterPixelConversion::convertPoint(cam_,points3D_inner_[i].get_x(),points3D_inner_[i].get_y(),model_inner_corner[i]);

      if(cmd.get_verbose()){
        std::cout << "model inner corner: (" << model_inner_corner[i].get_i() << "," << model_inner_corner[i].get_j() << ")" << std::endl;
      }
    }

    try{
      if(cmd.get_tracker_type()==CmdLine::MBT){
        vpMbEdgeTracker* me_tracker = dynamic_cast<vpMbEdgeTracker*>(tracker_);
        me_tracker->resetTracker();
        me_tracker->loadConfigFile(cmd.get_xml_file().c_str() );
        me_tracker->loadModel(cmd.get_wrl_file().c_str());
      }

      tracker_->initFromPose(Igray_,cMo_);

      tracker_->track(Igray_); // track the object on this image
      tracker_->getPose(cMo_); // get the pose
      tracker_->setCovarianceComputation(true);
      for(int i=0;i<cmd.get_mbt_convergence_steps();i++){
        tracker_->track(Igray_); // track the object on this image
        tracker_->getPose(cMo_); // get the pose
      }
    }catch(vpException& e){
      std::cout << "Tracking failed" << std::endl;
      std::cout << e.getStringMessage() << std::endl;
      return false;
    }
    //vpDisplay::getClick(*I_);
    return true;
  }

  bool Tracker_:: mbt_success(input_ready const& evt){
    iter_ = evt.frame;
    try{
      LogFileWriter writer(varfile_); //the destructor of this class will act as a finally statement
      vpImageConvert::convert(evt.I,Igray_);
      tracker_->track(Igray_); // track the object on this image
      tracker_->getPose(cMo_);
      vpMatrix mat = tracker_->getCovarianceMatrix();
      if(cmd.using_var_file()){
        writer.write(iter_);
        for(int i=0;i<mat.getRows();i++)
          writer.write(mat[i][i]);
      }
      if(cmd.using_var_limit())
        for(int i=0; i<6; i++)
          if(mat[i][i]>cmd.get_var_limit())
            return false;
      if(cmd.using_hinkley())
        for(int i=0; i<6; i++){
          if(hink_[i].testDownUpwardJump(mat[i][i]) != vpHinkley::noJump){
            writer.write(mat[i][i]);
            if(cmd.get_verbose())
              std::cout << "Hinkley:detected jump!" << std::endl;
            return false;
          }
        }
      if(cmd.using_var_file() && cmd.using_mbt_dynamic_range())
        writer.write(tracker_me_config_.getRange());



      for(int i=0;i<mat.getRows();i++)
        statistics.var(mat[i][i]);

      if(mat.getRows() == 6){ //if the covariance matrix is set
        statistics.var_x(mat[0][0]);
        statistics.var_y(mat[1][1]);
        statistics.var_z(mat[2][2]);
        statistics.var_wx(mat[3][3]);
        statistics.var_wy(mat[4][4]);
        statistics.var_wz(mat[5][5]);
      }

      if(cmd.using_var_file() && cmd.log_pose()){
        vpPoseVector p(cMo_);
        for(int i=0;i<p.getRows();i++)
          writer.write(p[i]);
      }

      if(cmd.using_adhoc_recovery() || cmd.log_checkpoints()){
        for(int p=0;p<points3D_middle_.size();p++){
          vpPoint& point3D = points3D_middle_[p];

          double _u=0.,_v=0.,_u_inner=0.,_v_inner=0.;
          point3D.project(cMo_);
          vpMeterPixelConversion::convertPoint(cam_,point3D.get_x(),point3D.get_y(),_u,_v);
          vpMeterPixelConversion::convertPoint(cam_,points3D_inner_[p].get_x(),points3D_inner_[p].get_y(),_u_inner,_v_inner);

          boost::accumulators::accumulator_set<
                  unsigned char,
                  boost::accumulators::stats<
                    boost::accumulators::tag::median(boost::accumulators::with_p_square_quantile)
                  >
                > acc;

          int region_width= std::max((int)(std::abs(_u-_u_inner)*cmd.get_adhoc_recovery_size()),1);
          int region_height=std::max((int)(std::abs(_v-_v_inner)*cmd.get_adhoc_recovery_size()),1);
          int u=(int)_u;
          int v=(int)_v;
          for(int i=std::max(u-region_width,0);
              i<std::min(u+region_width,(int)evt.I.getWidth());
              i++){
            for(int j=std::max(v-region_height,0);
                j<std::min(v+region_height,(int)evt.I.getHeight());
                j++){
              acc(Igray_[j][i]);
              statistics.checkpoints(Igray_[j][i]);
            }
          }
          double checkpoints_median = boost::accumulators::median(acc);
          if(cmd.using_var_file() && cmd.log_checkpoints())
            writer.write(checkpoints_median);
          if( cmd.using_adhoc_recovery() && (unsigned int)checkpoints_median>cmd.get_adhoc_recovery_treshold() )
            return false;
        }


      }

    }catch(vpException& e){
      std::cout << "Tracking lost" << std::endl;
      return false;
    }
    return true;
  }

  void Tracker_:: track_model(input_ready const& evt){
    this->cam_ = evt.cam_;
    std::vector<cv::Point> points;
    I_ = _I = &(evt.I);
    vpImageConvert::convert(evt.I,Igray_);
    boost::accumulators::accumulator_set<
                      double,
                      boost::accumulators::stats<
                        boost::accumulators::tag::mean
                      >
                    > acc;
    for(int i=0;i<points3D_outer_.size();i++){
      points3D_outer_[i].project(cMo_);
      points3D_inner_[i].project(cMo_);

      double u=0.,v=0.,u_inner=0.,v_inner=0;
      vpMeterPixelConversion::convertPoint(cam_,points3D_outer_[i].get_x(),points3D_outer_[i].get_y(),u,v);
      vpMeterPixelConversion::convertPoint(cam_,points3D_inner_[i].get_x(),points3D_inner_[i].get_y(),u_inner,v_inner);
      points.push_back(cv::Point(u,v));

      acc(std::abs(u-u_inner));
      acc(std::abs(v-v_inner));
    }

    if(cmd.using_mbt_dynamic_range()){
      int range = (const unsigned int)(boost::accumulators::mean(acc)*cmd.get_mbt_dynamic_range());

      vpMbEdgeTracker *tracker_me = dynamic_cast<vpMbEdgeTracker*>(tracker_);
      if(tracker_me){
        tracker_me->getMovingEdge(tracker_me_config_);
        tracker_me_config_.setRange(range);
        tracker_me->setMovingEdge(tracker_me_config_);
      }else
        std::cout << "error: could not init moving edges on tracker that doesn't support them." << std::endl;
    }

    cvTrackingBox_ = cv::boundingRect(cv::Mat(points));
    int s_x = cvTrackingBox_.x,
        s_y = cvTrackingBox_.y,
        d_x = cvTrackingBox_.x + cvTrackingBox_.width,
        d_y = cvTrackingBox_.y + cvTrackingBox_.height;
    s_x = std::max(s_x,0);
    s_y = std::max(s_y,0);
    d_x = std::min(d_x,(int)evt.I.getWidth());
    d_y = std::min(d_y,(int)evt.I.getHeight());
    cvTrackingBox_.x = s_x;
    cvTrackingBox_.y = s_y;
    cvTrackingBox_.width = d_x - s_x;
    cvTrackingBox_.height = d_y - s_y;
    vpTrackingBox_.setRect(cvTrackingBox_.x,cvTrackingBox_.y,cvTrackingBox_.width,cvTrackingBox_.height);
  }

  Tracker_::statistics_t& Tracker_:: get_statistics(){
    return statistics;
  }

  void Tracker_:: set_flush_display(bool val){
    flush_display_ = val;
  }

  bool Tracker_:: get_flush_display(){
    return flush_display_;
  }
}

