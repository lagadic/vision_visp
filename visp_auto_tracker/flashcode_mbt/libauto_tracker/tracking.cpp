#include "ros/ros.h"
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
#include <visp/vpMbKltTracker.h>
#include <visp/vpMbEdgeTracker.h>

#include "logfilewriter.hpp"

namespace tracking{

#if VISP_VERSION_INT < VP_VERSION_INT(2,10,0)
  Tracker_:: Tracker_(CmdLine& cmd, detectors::DetectorBase* detector,vpMbTracker* tracker,bool flush_display) :
#else
  Tracker_:: Tracker_(CmdLine& cmd, vpDetectorBase* detector,vpMbTracker* tracker,bool flush_display) :
#endif
  cmd(cmd),
      iter_(0),
      flashcode_center_(640/2,480/2),
      detector_(detector),
      tracker_(tracker),
      flush_display_(flush_display){
    std::cout << "starting tracker" << std::endl;
    cvTrackingBox_init_ = false;
    cvTrackingBox_.x = 0;
    cvTrackingBox_.y = 0;
    cvTrackingBox_.width = 0;
    cvTrackingBox_.height = 0;

    // Retrieve camera parameters comming from camera_info message in order to update them after loadConfigFile()
    tracker_->getCameraParameters(cam_); // init camera parameters

    points3D_inner_ = cmd.get_inner_points_3D();
    points3D_outer_ = cmd.get_outer_points_3D();
    outer_points_3D_bcp_ = cmd.get_outer_points_3D();
    if(cmd.using_adhoc_recovery() || cmd.log_checkpoints()){
      for(unsigned int i=0;i<points3D_outer_.size();i++){
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
      varfile_ << "#These are variances and other data from the model based tracker in gnuplot format" << std::endl;
      if(cmd.using_hinkley())
        varfile_ << "iteration\tvar_x\var_y\tvar_z\tvar_wx\tvar_wy\var_wz";
      if(cmd.using_mbt_dynamic_range())
        varfile_ << "\tmbt_range";
      if(cmd.log_pose())
        varfile_ << "\tpose_tx\tpose_ty\tpose_tz\tpose_rx\tpose_ry\tpose_rz";
      if(cmd.log_checkpoints())
        varfile_ << "\tcheckpoint_median";

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

    tracker_->loadConfigFile(cmd.get_xml_file() ); // Load the configuration of the tracker
    tracker_->loadModel(cmd.get_mbt_cad_file()); // load the 3d model, to read .wrl model the 3d party library coin is required, if coin is not installed .cao file can be used.
    tracker_->setCameraParameters(cam_); // Set the good camera parameters coming from camera_info message
  }

#if VISP_VERSION_INT < VP_VERSION_INT(2,10,0)
  detectors::DetectorBase& Tracker_:: get_detector(){
#else
  vpDetectorBase& Tracker_:: get_detector(){
#endif
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
#if VISP_VERSION_INT < VP_VERSION_INT(2,10,0)
    //this->cam_ = evt.cam_;

    cv::Mat rgba = cv::Mat((int)evt.I.getRows(), (int)evt.I.getCols(), CV_8UC4, (void*)evt.I.bitmap);

    cv::Mat bgr = cv::Mat((int)evt.I.getRows(), (int)evt.I.getCols(), CV_8UC3);
    cv::Mat alpha((int)evt.I.getRows(), (int)evt.I.getCols(), CV_8UC1);

    cv::Mat out[] = {bgr, alpha};
    // rgba[0] -> bgr[2], rgba[1] -> bgr[1],
    // rgba[2] -> bgr[0], rgba[3] -> alpha[0]
    int from_to[] = { 0,2,  1,1,  2,0,  3,3 };
    cv::mixChannels(&rgba, 1, out, 2, from_to, 4);

    //vpImageConvert::convert(evt.I,bgr);

    return detector_->detect(bgr,cmd.get_dmx_timeout(),0,0);
#else
    vpImageConvert::convert(evt.I, Igray_);
    detector_->detect(Igray_);
    if (detector_->getNbObjects()) {
      if (cmd.get_code_message().empty()) {
        cmd.set_code_message_index(0); // we retain the largest code at index 0
        return true;
      }
      else {
        for (size_t i=0; i<detector_->getNbObjects(); i++) {
          if (detector_->getMessage(i) == cmd.get_code_message()) {
            cmd.set_code_message_index(i);
            ROS_WARN_STREAM("Code with message \"" << cmd.get_code_message() << "\" found");
            return true;
          }
        }
        ROS_WARN_STREAM("Code with message \"" << cmd.get_code_message() << "\" not found");
        return false;
      }
    }
    return false;
#endif
  }

  /*
   * Detect flashcode in region delimited by the outer points of the model
   * The timeout is the default timeout times the surface ratio
   */
  bool Tracker_:: flashcode_redetected(input_ready const& evt){
#if VISP_VERSION_INT < VP_VERSION_INT(2,10,0)
    //this->cam_ = evt.cam_;

    //vpImageConvert::convert(evt.I,cvI);
    cv::Mat rgba = cv::Mat((int)evt.I.getRows(), (int)evt.I.getCols(), CV_8UC4, (void*)evt.I.bitmap);

    cv::Mat bgr = cv::Mat((int)evt.I.getRows(), (int)evt.I.getCols(), CV_8UC3);
    cv::Mat alpha((int)evt.I.getRows(), (int)evt.I.getCols(), CV_8UC1);

    cv::Mat out[] = {bgr, alpha};
    // rgba[0] -> bgr[2], rgba[1] -> bgr[1],
    // rgba[2] -> bgr[0], rgba[3] -> alpha[0]
    int from_to[] = { 0,2,  1,1,  2,0,  3,3 };
    cv::mixChannels(&rgba, 1, out, 2, from_to, 4);

    if (cvTrackingBox_init_)
    {
      cv::Mat subImage = cv::Mat(bgr,get_tracking_box<cv::Rect>()).clone();

      double timeout = cmd.get_dmx_timeout()*(double)(get_tracking_box<cv::Rect>().width*get_tracking_box<cv::Rect>().height)/(double)(bgr.cols*bgr.rows);
      return detector_->detect(subImage,(unsigned int)timeout,get_tracking_box<cv::Rect>().x,get_tracking_box<cv::Rect>().y);
    }
    else
    {
      return detector_->detect(bgr,cmd.get_dmx_timeout(),0,0);
    }
#else
    // TODO, use boundig box as for ViSP < 2.10.0
    vpImageConvert::convert(evt.I, Igray_);
    detector_->detect(Igray_);
    if (detector_->getNbObjects()) {
      if (cmd.get_code_message().empty()) {
        cmd.set_code_message_index(0); // we retain the largest code at index 0
        return true;
      }
      else {
        for (size_t i=0; i<detector_->getNbObjects(); i++) {
          if (detector_->getMessage(i) == cmd.get_code_message()) {
            cmd.set_code_message_index(i);
            ROS_WARN_STREAM("Code with message \"" << cmd.get_code_message() << "\" found");
            return true;
          }
        }
        ROS_WARN_STREAM("Code with message \"" << cmd.get_code_message() << "\" not found");
        return false;
      }
    }
    return false;
#endif
  }

  void Tracker_:: find_flashcode_pos(input_ready const& evt){
    this->cam_ = evt.cam_;

#if VISP_VERSION_INT < VP_VERSION_INT(2,10,0)
    std::vector<cv::Point> polygon = detector_->get_polygon();
    double centerX = (double)(polygon[0].x+polygon[1].x+polygon[2].x+polygon[3].x)/4.;
    double centerY = (double)(polygon[0].y+polygon[1].y+polygon[2].y+polygon[3].y)/4.;
    vpPixelMeterConversion::convertPoint(cam_, flashcode_center_, centerX, centerY);

    for(unsigned int i=0;i<f_.size();i++){
      double x=0, y=0;
      vpImagePoint poly_pt(polygon[i].y,polygon[i].x);

      vpPixelMeterConversion::convertPoint(cam_, poly_pt, x, y);
      f_[i].set_x(x);
      f_[i].set_y(y);
    }
#else
    // TODO: add a parameter to be able to select the QRcode from it's message
    // For the moment we get the position of the first code that is the largest in the image
    std::vector< std::vector< vpImagePoint > > polygons = detector_->getPolygon();
    std::vector< vpImagePoint > polygon(4);
    if (polygons.size())
      polygon = polygons[0];

    // TODO: remove flashcode_center_, centerX, centerY that are not used
    //double centerX = cog.get_u();
    //double centerY = cog.get_v();
    //vpPixelMeterConversion::convertPoint(cam_, flashcode_center_, centerX, centerY);

    for(unsigned int i=0;i<f_.size();i++){
      double x=0, y=0;

      vpPixelMeterConversion::convertPoint(cam_, polygon[i], x, y);
      f_[i].set_x(x);
      f_[i].set_y(y);
    }
#endif

    I_ = _I = &(evt.I);
  }


  bool Tracker_:: model_detected(msm::front::none const&){
    vpImageConvert::convert(*I_,Igray_);
    vpPose pose;

    for(unsigned int i=0;i<f_.size();i++)
      pose.addPoint(f_[i]);

    try {
      vpHomogeneousMatrix cMo_dem;
      vpHomogeneousMatrix cMo_lag;
      pose.computePose(vpPose::DEMENTHON, cMo_dem);
      pose.computePose(vpPose::LAGRANGE, cMo_lag);
      double residual_dem = pose.computeResidual(cMo_dem);
      double residual_lag = pose.computeResidual(cMo_lag);
      if (residual_dem < residual_lag)
        cMo_ = cMo_dem;
      else
        cMo_ = cMo_lag;
      pose.computePose(vpPose::VIRTUAL_VS,cMo_);
      //vpDisplay::displayFrame(*I_,cMo_,cam_,0.01,vpColor::none,2);
    }
    catch(vpException& e) {
      std::cout << "Pose computation failed: " << e.getStringMessage() << std::endl;
      return false;
    }

    std::vector<vpImagePoint> model_inner_corner(4);
    std::vector<vpImagePoint> model_outer_corner(4);
    for(unsigned int i=0;i<4;i++){
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
      tracker_->resetTracker();
      tracker_->loadConfigFile(cmd.get_xml_file() );
      tracker_->loadModel(cmd.get_mbt_cad_file()); // load the 3d model, to read .wrl model the 3d party library coin is required, if coin is not installed .cao file can be used.
      tracker_->setCameraParameters(cam_);
      {
          vpCameraParameters cam;
          tracker_->getCameraParameters(cam);
          if (cam.get_px() != 558) ROS_INFO_STREAM("detection Camera parameters: \n" << cam_);
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
    return true;
  }

  bool Tracker_:: mbt_success(input_ready const& evt){
    iter_ = evt.frame;
    this->cam_ = evt.cam_;

    try{
      LogFileWriter writer(varfile_); //the destructor of this class will act as a finally statement
      vpImageConvert::convert(evt.I,Igray_);

      tracker_->track(Igray_); // track the object on this image
      tracker_->getPose(cMo_);
      covariance_ = tracker_->getCovarianceMatrix();
      if(cmd.using_var_file()){
        writer.write(iter_);
        for(unsigned int i=0;i<covariance_.getRows();i++)
          writer.write(covariance_[i][i]);
      }
      if(cmd.using_var_limit())
        for(unsigned int i=0; i<6; i++)
          if(covariance_[i][i]>cmd.get_var_limit())
            return false;
      if(cmd.using_hinkley())
        for(unsigned int i=0; i<6; i++){
          if(hink_[i].testDownUpwardJump(covariance_[i][i]) != vpHinkley::noJump){
            writer.write(covariance_[i][i]);
            if(cmd.get_verbose())
              std::cout << "Hinkley:detected jump!" << std::endl;
            return false;
          }
        }
      if(cmd.using_var_file() && cmd.using_mbt_dynamic_range())
        writer.write(tracker_me_config_.getRange());

      for(unsigned int i=0;i<covariance_.getRows();i++) {
        statistics.var(covariance_[i][i]);
      }

      if(covariance_.getRows() == 6){ //if the covariance matrix is set
        statistics.var_x(covariance_[0][0]);
        statistics.var_y(covariance_[1][1]);
        statistics.var_z(covariance_[2][2]);
        statistics.var_wx(covariance_[3][3]);
        statistics.var_wy(covariance_[4][4]);
        statistics.var_wz(covariance_[5][5]);
      }

      if(cmd.using_var_file() && cmd.log_pose()){
        vpPoseVector p(cMo_);
        for(unsigned int i=0;i<p.getRows();i++)
          writer.write(p[i]);
      }

      if(cmd.using_adhoc_recovery() || cmd.log_checkpoints()){
        for(unsigned int p=0;p<points3D_middle_.size();p++){
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

    for(unsigned int i=0;i<points3D_outer_.size();i++){
      points3D_outer_[i].project(cMo_);
      points3D_inner_[i].project(cMo_);

      double u=0.,v=0.,u_inner=0.,v_inner=0;
      vpMeterPixelConversion::convertPoint(cam_,points3D_outer_[i].get_x(),points3D_outer_[i].get_y(),u,v);
      vpMeterPixelConversion::convertPoint(cam_,points3D_inner_[i].get_x(),points3D_inner_[i].get_y(),u_inner,v_inner);

      acc(std::abs(u-u_inner));
      acc(std::abs(v-v_inner));

      // To avoid OpenCV exception that may occur when creating cv::boundingRect() from the points,
      // we ensure that the coordinates of the points remain in the image.
      u = std::max(u,0.);
      u = std::min(u,(double)evt.I.getWidth()-1);
      v = std::max(v,0.);
      v = std::min(v,(double)evt.I.getHeight()-1);
      points.push_back(cv::Point(u,v));
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
    cvTrackingBox_init_ = true;
    cvTrackingBox_ = cv::boundingRect(cv::Mat(points));

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

  void
  Tracker_::updateMovingEdgeSites(visp_tracker::MovingEdgeSitesPtr sites)
  {
    if (!sites)
      return;

    std::list<vpMbtDistanceLine*> linesList;

    if(cmd.get_tracker_type() != CmdLine::KLT) // For mbt and hybrid
      dynamic_cast<vpMbEdgeTracker*>(tracker_)->getLline(linesList, 0);

    std::list<vpMbtDistanceLine*>::iterator linesIterator = linesList.begin();
    if (linesList.empty())
      ROS_DEBUG_THROTTLE(10, "no distance lines");
    bool noVisibleLine = true;
    for (; linesIterator != linesList.end(); ++linesIterator)
    {
      vpMbtDistanceLine* line = *linesIterator;

#if VISP_VERSION_INT >= VP_VERSION_INT(3,0,0) // ViSP >= 3.0.0
      if (line && line->isVisible() && ! line->meline.empty())
#else
      if (line && line->isVisible() && line->meline)
#endif
      {
#if VISP_VERSION_INT >= VP_VERSION_INT(3,0,0) // ViSP >= 3.0.0
        for(unsigned int a = 0 ; a < line->meline.size() ; a++)
        {
          if(line->meline[a] != NULL) {
            std::list<vpMeSite>::const_iterator sitesIterator = line->meline[a]->getMeList().begin();
            if (line->meline[a]->getMeList().empty())
              ROS_DEBUG_THROTTLE(10, "no moving edge for a line");
            for (; sitesIterator != line->meline[a]->getMeList().end(); ++sitesIterator)
#elif VISP_VERSION_INT >= VP_VERSION_INT(2,10,0) // ViSP >= 2.10.0
        if (line->meline->getMeList().empty()) {
          ROS_DEBUG_THROTTLE(10, "no moving edge for a line");
        }
        std::list<vpMeSite>::const_iterator sitesIterator = line->meline->getMeList().begin();

        for (; sitesIterator != line->meline->getMeList().end(); ++sitesIterator)
#else
        if (line->meline->list.empty()) {
          ROS_DEBUG_THROTTLE(10, "no moving edge for a line");
        }
        std::list<vpMeSite>::const_iterator sitesIterator = line->meline->list.begin();

        for (; sitesIterator != line->meline->list.end(); ++sitesIterator)
#endif
        {
          visp_tracker::MovingEdgeSite movingEdgeSite;
          movingEdgeSite.x = sitesIterator->ifloat;
          movingEdgeSite.y = sitesIterator->jfloat;
#if VISP_VERSION_INT < VP_VERSION_INT(2,10,0)// ViSP < 2.10.0
          movingEdgeSite.suppress = sitesIterator->suppress;
#endif
          sites->moving_edge_sites.push_back (movingEdgeSite);
        }
        noVisibleLine = false;
      }
#if VISP_VERSION_INT >= VP_VERSION_INT(3,0,0) // ViSP >= 3.0.0
      }
    }
#endif
    }
    if (noVisibleLine)
      ROS_DEBUG_THROTTLE(10, "no distance lines");
  }

  void
  Tracker_::updateKltPoints(visp_tracker::KltPointsPtr klt)
  {
    if (!klt)
      return;

#if VISP_VERSION_INT < VP_VERSION_INT(2,10,0)// ViSP < 2.10.0
    vpMbHiddenFaces<vpMbtKltPolygon> *poly_lst;
    std::map<int, vpImagePoint> *map_klt;

    if(cmd.get_tracker_type() != CmdLine::MBT) // For klt and hybrid
      poly_lst = &dynamic_cast<vpMbKltTracker*>(tracker_)->getFaces();

    for(unsigned int i = 0 ; i < poly_lst->size() ; i++)
    {
      if((*poly_lst)[i])
      {
        map_klt = &((*poly_lst)[i]->getCurrentPoints());

        if(map_klt->size() > 3)
        {
          for (std::map<int, vpImagePoint>::iterator it=map_klt->begin(); it!=map_klt->end(); ++it)
          {
            visp_tracker::KltPoint kltPoint;
            kltPoint.id = it->first;
            kltPoint.i = it->second.get_i();
            kltPoint.j = it->second.get_j();
            klt->klt_points_positions.push_back (kltPoint);
          }
        }
      }
    }
#else // ViSP >= 2.10.0
    std::list<vpMbtDistanceKltPoints*> *poly_lst;
    std::map<int, vpImagePoint> *map_klt;

    if(cmd.get_tracker_type() != CmdLine::MBT) { // For klt and hybrid
      poly_lst = &dynamic_cast<vpMbKltTracker*>(tracker_)->getFeaturesKlt();

      for(std::list<vpMbtDistanceKltPoints*>::const_iterator it=poly_lst->begin(); it!=poly_lst->end(); ++it){
        map_klt = &((*it)->getCurrentPoints());

        if((*it)->polygon->isVisible()){
        if(map_klt->size() > 3)
          {
            for (std::map<int, vpImagePoint>::iterator it=map_klt->begin(); it!=map_klt->end(); ++it)
            {
              visp_tracker::KltPoint kltPoint;
              kltPoint.id = it->first;
              kltPoint.i = it->second.get_i();
              kltPoint.j = it->second.get_j();
              klt->klt_points_positions.push_back (kltPoint);
            }
          }
        }
      }
    }
#endif
  }
}

