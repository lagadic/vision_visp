#ifndef __EVENTS_H__
#define __EVENTS_H__
#include "cv.h"
// back-end
#include <boost/msm/back/state_machine.hpp>
//front-end
#include <boost/msm/front/state_machine_def.hpp>
#include <visp/vpImage.h>
#include <visp/vpRGBa.h>
#include <visp/vpMeterPixelConversion.h>
#include <visp/vpPlot.h>
#include <visp/vpRect.h>
#include <visp/vpDisplay.h>
#include <vector>
#include <cassert>
#include <fstream>
#include <boost/thread.hpp>
#include "events.h"
#include <ros/ros.h>


namespace msm = boost::msm;

namespace tracking{
  struct WaitingForInput : public msm::front::state<>{
      template <class Event, class Fsm>
      void on_entry(Event const&, Fsm& fsm){
        if(fsm.get_cmd().get_verbose())
          std::cout <<"entering: WaitingForInput" << std::endl;
      }
      template <class Event, class Fsm>
      void on_exit(Event const& evt, Fsm& fsm){
        if(fsm.get_cmd().get_verbose())
          std::cout <<"leaving: WaitingForInput" << std::endl;
        if(fsm.get_flush_display()){
          vpDisplay::display(evt.I);
          vpDisplay::flush(evt.I);
        }
      }

  };

  struct Finished : public msm::front::state<>{
    template <class Event, class Fsm>
    void on_entry(Event const& evt, Fsm& fsm){
      if(fsm.get_cmd().get_verbose())
      {
        typename Fsm::statistics_t& statistics = fsm.get_statistics();
        std::cout << "statistics:" << std::endl;
        std::cout << "\tglobal:" << std::endl;
        std::cout << "\t\tmedian:" << boost::accumulators::median(statistics.var) << std::endl;
        std::cout << "\t\tmean:" << boost::accumulators::mean(statistics.var) << std::endl;
        std::cout << "\t\tmax:" << boost::accumulators::max(statistics.var) << std::endl;

        std::cout << "\tX:" << std::endl;
        std::cout << "\t\tmedian:" << boost::accumulators::median(statistics.var_x) << std::endl;
        std::cout << "\t\tmean:" << boost::accumulators::mean(statistics.var_x) << std::endl;
        std::cout << "\t\tmax:" << boost::accumulators::max(statistics.var_x) << std::endl;

        std::cout << "\tY:" << std::endl;
        std::cout << "\t\tmedian:" << boost::accumulators::median(statistics.var_y) << std::endl;
        std::cout << "\t\tmean:" << boost::accumulators::mean(statistics.var_y) << std::endl;
        std::cout << "\t\tmax:" << boost::accumulators::max(statistics.var_y) << std::endl;

        std::cout << "\tZ:" << std::endl;
        std::cout << "\t\tmedian:" << boost::accumulators::median(statistics.var_z) << std::endl;
        std::cout << "\t\tmean:" << boost::accumulators::mean(statistics.var_z) << std::endl;
        std::cout << "\t\tmax:" << boost::accumulators::max(statistics.var_z) << std::endl;

        std::cout << "\tW_X:" << std::endl;
        std::cout << "\t\tmedian:" << boost::accumulators::median(statistics.var_wx) << std::endl;
        std::cout << "\t\tmean:" << boost::accumulators::mean(statistics.var_wx) << std::endl;
        std::cout << "\t\tmax:" << boost::accumulators::max(statistics.var_wx) << std::endl;

        std::cout << "\tW_Y:" << std::endl;
        std::cout << "\t\tmedian:" << boost::accumulators::median(statistics.var_wy) << std::endl;
        std::cout << "\t\tmean:" << boost::accumulators::mean(statistics.var_wy) << std::endl;
        std::cout << "\t\tmax:" << boost::accumulators::max(statistics.var_wy) << std::endl;

        std::cout << "\tW_Z:" << std::endl;
        std::cout << "\t\tmedian:" << boost::accumulators::median(statistics.var_wz) << std::endl;
        std::cout << "\t\tmean:" << boost::accumulators::mean(statistics.var_wz) << std::endl;
        std::cout << "\t\tmax:" << boost::accumulators::max(statistics.var_wz) << std::endl;
      }
    }
  };

  struct DetectFlashcodeGeneric : public msm::front::state<>
  {
      vpImagePoint corner0;
      vpImagePoint corner1;
      vpImagePoint corner2;
      vpImagePoint corner3;
      virtual vpColor getColor() = 0;
      template <class Fsm>
      void on_entry(finished const& evt, Fsm& fsm){}

      template <class Fsm>
      void on_exit(finished const& evt, Fsm& fsm){}

      template <class Event, class Fsm>
      void on_entry(Event const&, Fsm& fsm)
      {
        if(fsm.get_cmd().get_verbose())
          std::cout <<"entering: DetectFlashcode" << std::endl;
      }
      template <class Event, class Fsm>
      void on_exit(Event const& evt, Fsm& fsm)
      {
        if(fsm.get_cmd().get_verbose())
          std::cout <<"leaving: DetectFlashcode" << std::endl;
        if(fsm.get_flush_display()) {
          vpDisplay::display(evt.I);
        }
#if VISP_VERSION_INT < VP_VERSION_INT(2,10,0)
        std::vector<cv::Point>& polygon = fsm.get_detector().get_polygon();
        if(polygon.size()!=4) {
          if(fsm.get_flush_display()) vpDisplay::flush(evt.I);
          return;
        }
        corner0 = vpImagePoint (polygon[0].y,polygon[0].x);
        corner1 = vpImagePoint (polygon[1].y,polygon[1].x);
        corner2 = vpImagePoint (polygon[2].y,polygon[2].x);
        corner3 = vpImagePoint (polygon[3].y,polygon[3].x);
#else
        // TODO: add a parameter to be able to select the QRcode from it's message
        // For the moment we get the position of the first code that is the largest in the image
        std::vector< std::vector< vpImagePoint > > polygons = fsm.get_detector().getPolygon();
        std::vector< vpImagePoint > polygon(4);
        if (polygons.size())
          polygon = polygons[0];
        if(polygon.size()!=4) {
          if(fsm.get_flush_display()) vpDisplay::flush(evt.I);
          return;
        }
        corner0 = polygon[0];
        corner1 = polygon[1];
        corner2 = polygon[2];
        corner3 = polygon[3];
#endif

#if VISP_VERSION_INT < VP_VERSION_INT(2,10,0)
        if(0){//fsm.get_flush_display()){
          vpDisplay::displayRectangle(evt.I,fsm.template get_tracking_box< vpRect > (),getColor(),false,2);
          if(polygon.size()==0){
            vpDisplay::displayCharString(evt.I,vpImagePoint(0,0),"TRACKING LOST",vpColor::red);
            vpDisplay::flush(evt.I);
            return;
          }

          std::vector<std::pair<cv::Point,cv::Point> >& lines = fsm.get_detector().get_lines();
          for(std::vector<std::pair<cv::Point,cv::Point> >::iterator i = lines.begin();
              i!=lines.end();
              i++
          ){
            vpDisplay::displayLine(evt.I,vpImagePoint(i->first.y,i->first.x),vpImagePoint(i->second.y,i->second.x),getColor(),2);
          }
          vpDisplay::displayCharString(evt.I,corner0,"1",vpColor::blue);
          vpDisplay::displayCharString(evt.I,corner1,"2",vpColor::yellow);
          vpDisplay::displayCharString(evt.I,corner2,"3",vpColor::cyan);
          vpDisplay::displayCharString(evt.I,corner3,"4",vpColor::darkRed);

        }
#endif
        vpDisplay::flush(evt.I);
      }
  };

  struct DetectFlashcode: public DetectFlashcodeGeneric {
    vpColor getColor(){ return vpColor::green; }
  };
  struct ReDetectFlashcode: public DetectFlashcodeGeneric {
    template <class Event, class Fsm>
    void on_entry(Event const&, Fsm& fsm)
    {
      if(fsm.get_cmd().get_verbose())
        std::cout <<"entering: ReDetectFlashcode" << std::endl;
    }
    vpColor getColor(){ return vpColor::orange; }
  };

  struct DetectModel : public msm::front::state<>
  {
      std::vector<vpImagePoint> model_inner_corner;
      std::vector<vpImagePoint> model_outer_corner;
      vpHomogeneousMatrix cMo;

      DetectModel() : model_inner_corner(4),model_outer_corner(4){}
      template <class Fsm>
      void on_entry(finished const& evt, Fsm& fsm){}

      template <class Fsm>
      void on_exit(finished const& evt, Fsm& fsm){}

      template <class Event, class Fsm>
      void on_entry(Event const&, Fsm& fsm)
      {
        if(fsm.get_cmd().get_verbose())
          std::cout <<"entering: DetectModel" << std::endl;
      }
      template <class Event, class Fsm>
      void on_exit(Event const& evt, Fsm& fsm)
      {
        if(fsm.get_cmd().get_verbose())
          std::cout <<"leaving: DetectModel" << std::endl;
        std::vector<vpPoint>& points3D_inner = fsm.get_points3D_inner();
        std::vector<vpPoint>& points3D_outer = fsm.get_points3D_outer();

        fsm.get_mbt().getPose(cMo);

        for(unsigned int i=0;i<4;i++){
          vpMeterPixelConversion::convertPoint(fsm.get_cam(),points3D_outer[i].get_x(),points3D_outer[i].get_y(),model_outer_corner[i]);
          vpMeterPixelConversion::convertPoint(fsm.get_cam(),points3D_inner[i].get_x(),points3D_inner[i].get_y(),model_inner_corner[i]);
        }
        if(fsm.get_flush_display()){
          vpImage<vpRGBa>& I = fsm.get_I();
          vpDisplay::displayCharString(I,model_inner_corner[0],"mi1",vpColor::blue);
          vpDisplay::displayCross(I,model_inner_corner[0],2,vpColor::blue,2);
          vpDisplay::displayCharString(I,model_inner_corner[1],"mi2",vpColor::yellow);
          vpDisplay::displayCross(I,model_inner_corner[1],2,vpColor::yellow,2);
          vpDisplay::displayCharString(I,model_inner_corner[2],"mi3",vpColor::cyan);
          vpDisplay::displayCross(I,model_inner_corner[2],2,vpColor::cyan,2);
          vpDisplay::displayCharString(I,model_inner_corner[3],"mi4",vpColor::darkRed);
          vpDisplay::displayCross(I,model_inner_corner[3],2,vpColor::darkRed,2);

          vpDisplay::displayCharString(I,model_outer_corner[0],"mo1",vpColor::blue);
          vpDisplay::displayCross(I,model_outer_corner[0],2,vpColor::blue,2);
          vpDisplay::displayCharString(I,model_outer_corner[1],"mo2",vpColor::yellow);
          vpDisplay::displayCross(I,model_outer_corner[1],2,vpColor::yellow,2);
          vpDisplay::displayCharString(I,model_outer_corner[2],"mo3",vpColor::cyan);
          vpDisplay::displayCross(I,model_outer_corner[2],2,vpColor::cyan,2);
          vpDisplay::displayCharString(I,model_outer_corner[3],"mo4",vpColor::darkRed);
          vpDisplay::displayCross(I,model_outer_corner[3],2,vpColor::darkRed,2);

          try {
            fsm.get_mbt().display(I, cMo, fsm.get_cam(), vpColor::blue, 1);// display the model at the computed pose.
          }
          catch(vpException& e)
          {
            std::cout << "Cannot display the model" << std::endl;
          }

          vpDisplay::flush(I);
        }

      }
  };

  class TrackModel : public msm::front::state<>
  {
  private:
    vpPlot* plot_;
    int iter_;
  public:
    vpHomogeneousMatrix cMo;
    vpMatrix covariance;

    ~TrackModel(){
      delete plot_;
    }

    TrackModel() : plot_(NULL),iter_(0){}

    template <class Fsm>
    void on_entry(finished const& evt, Fsm& fsm){
      fsm.get_mbt().getPose(cMo);
      covariance = fsm.get_mbt().getCovarianceMatrix();
    }

    template <class Fsm>
    void on_exit(finished const& evt, Fsm& fsm){}

    template <class Event, class Fsm>
    void on_entry(Event const& evt, Fsm& fsm)
    {
      if(fsm.get_cmd().show_plot() && plot_ == NULL){
        plot_ = new vpPlot(1, 700, 700, 100, 200, "Variances");
        plot_->initGraph(0,7);
      }
    }
    template <class Event, class Fsm>
    void on_exit(Event const& evt, Fsm& fsm)
    {
      fsm.get_mbt().getPose(cMo);
      covariance = fsm.get_mbt().getCovarianceMatrix();
      if(fsm.get_flush_display()){
        vpDisplay::display(evt.I);

        fsm.get_mbt().display(evt.I, cMo, fsm.get_cam(), vpColor::red, 1);// display the model at the computed pose.
        vpDisplay::displayFrame(evt.I,cMo,fsm.get_cam(),.1,vpColor::none,2);
        if(fsm.get_cmd().using_adhoc_recovery() && fsm.get_cmd().get_adhoc_recovery_display()){
          for(unsigned int p=0;p<fsm.get_points3D_middle().size();p++){
            vpPoint& point3D = fsm.get_points3D_middle()[p];
            vpPoint& point3D_inner = fsm.get_points3D_inner()[p];
            double _u=0.,_v=0.,_u_inner=0.,_v_inner=0.;

            vpMeterPixelConversion::convertPoint(fsm.get_cam(),point3D.get_x(),point3D.get_y(),_u,_v);
            vpMeterPixelConversion::convertPoint(fsm.get_cam(),point3D_inner.get_x(),point3D_inner.get_y(),_u_inner,_v_inner);
            int region_width= std::max((int)(std::abs(_u-_u_inner)*fsm.get_cmd().get_adhoc_recovery_size()),1);
            int region_height=std::max((int)(std::abs(_v-_v_inner)*fsm.get_cmd().get_adhoc_recovery_size()),1);

            int u=(int)_u;
            int v=(int)_v;
            vpDisplay::displayRectangle(
                evt.I,
                vpImagePoint(
                    std::max(v-region_height,0),
                    std::max(u-region_width,0)
                ),
                vpImagePoint(
                    std::max(0, std::min(v+region_height,(int)evt.I.getHeight())),
                    std::max(0, std::min(u+region_width,(int)evt.I.getWidth()))
                ),
                vpColor::cyan,
                true
                );
          }
        }
        vpDisplay::flush(evt.I);

        if(fsm.get_cmd().show_plot()){
          if(fsm.get_cmd().using_var_limit())
            plot_->plot(0,6,iter_,(double)fsm.get_cmd().get_var_limit());
          for(unsigned int i=0;i<6;i++)
            plot_->plot(0,i,iter_,covariance[i][i]);
        }
      }

      iter_++;
    }
  };
}
#endif /* EVENTS_H_ */
