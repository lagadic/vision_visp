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



namespace msm = boost::msm;

namespace tracking{
  struct WaitingForInput : public msm::front::state<>{
      template <class Event, class Fsm>
      void on_entry(Event const&, Fsm& ){
        std::cout <<"entering: WaitingForInput" << std::endl;
      }
      template <class Event, class Fsm>
      void on_exit(Event const& evt, Fsm& fsm){
        std::cout <<"leaving: WaitingForInput" << std::endl;
        vpDisplay::display(evt.I);
        vpDisplay::flush(evt.I);
      }
  };

  struct Finished : public msm::front::state<>{
    template <class Event, class Fsm>
    void on_entry(Event const& evt, Fsm& fsm){
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
  };

  struct DetectFlashcodeGeneric : public msm::front::state<>
  {
      virtual vpColor getColor() = 0;
      template <class Fsm>
      void on_entry(finished const& evt, Fsm& fsm){}

      template <class Fsm>
      void on_exit(finished const& evt, Fsm& fsm){}

      template <class Event, class Fsm>
      void on_entry(Event const&, Fsm&)
      {
        std::cout <<"entering: DetectFlashcode" << std::endl;

      }
      template <class Event, class Fsm>
      void on_exit(Event const& evt, Fsm& fsm)
      {
        std::cout <<"leaving: DetectFlashcode" << std::endl;
        vpDisplay::display(evt.I);
        vpDisplay::displayRectangle(evt.I,fsm.template get_tracking_box< vpRect > (),getColor(),false,2);
        std::vector<cv::Point>& polygon = fsm.get_detector().get_polygon();
        if(polygon.size()==0){
          vpDisplay::displayCharString(evt.I,vpImagePoint(0,0),"TRACKING LOST",vpColor::red);
          vpDisplay::flush(evt.I);
          return;
        }

        const vpImagePoint corner0(polygon[0].y,polygon[0].x);
        const vpImagePoint corner1(polygon[1].y,polygon[1].x);
        const vpImagePoint corner2(polygon[2].y,polygon[2].x);
        const vpImagePoint corner3(polygon[3].y,polygon[3].x);

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

        vpDisplay::flush(evt.I);
      }
  };

  struct DetectFlashcode: public DetectFlashcodeGeneric {
    vpColor getColor(){ return vpColor::green; }
  };
  struct ReDetectFlashcode: public DetectFlashcodeGeneric {
    template <class Event, class Fsm>
    void on_entry(Event const&, Fsm&)
    {
      std::cout <<"entering: ReDetectFlashcode" << std::endl;

    }
    vpColor getColor(){ return vpColor::orange; }
  };

  struct DetectModel : public msm::front::state<>
  {
      template <class Fsm>
      void on_entry(finished const& evt, Fsm& fsm){}

      template <class Fsm>
      void on_exit(finished const& evt, Fsm& fsm){}

      template <class Event, class Fsm>
      void on_entry(Event const&, Fsm& )
      {std::cout <<"entering: DetectModel" << std::endl;}
      template <class Event, class Fsm>
      void on_exit(Event const& evt, Fsm& fsm)
      {
        std::cout <<"leaving: DetectModel" << std::endl;
        std::vector<vpPoint>& points3D_inner = fsm.get_points3D_inner();
        std::vector<vpPoint>& points3D_outer = fsm.get_points3D_outer();

        std::vector<vpImagePoint> model_inner_corner(4);
        std::vector<vpImagePoint> model_outer_corner(4);
        for(int i=0;i<4;i++){
          vpMeterPixelConversion::convertPoint(fsm.get_cam(),points3D_outer[i].get_x(),points3D_outer[i].get_y(),model_outer_corner[i]);
          vpMeterPixelConversion::convertPoint(fsm.get_cam(),points3D_inner[i].get_x(),points3D_inner[i].get_y(),model_inner_corner[i]);
        }

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

        vpHomogeneousMatrix cMo;
        fsm.get_mbt().getPose(cMo);
        fsm.get_mbt().display(I, cMo, fsm.get_cam(), vpColor::blue, 1);// display the model at the computed pose.
        vpDisplay::flush(I);
      }
  };

  class TrackModel : public msm::front::state<>
  {
  private:
    vpPlot* plot_;
    int iter_;
  public:
    ~TrackModel(){
      delete plot_;
    }

    TrackModel() : plot_(NULL),iter_(0){}

    template <class Fsm>
    void on_entry(finished const& evt, Fsm& fsm){}

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
      vpHomogeneousMatrix cMo;
      fsm.get_mbt().getPose(cMo);
      vpDisplay::display(evt.I);
      fsm.get_mbt().display(evt.I, cMo, fsm.get_cam(), vpColor::red, 1);// display the model at the computed pose.
      vpDisplay::displayFrame(evt.I,cMo,fsm.get_cam(),.3,vpColor::none,2);
      if(fsm.get_cmd().using_adhoc_recovery_ratio()){
        for(std::vector<vpPoint>::iterator point3D = fsm.get_points3D_middle().begin();
            point3D != fsm.get_points3D_middle().end();
            point3D++
            ){
          double _u=0.,_v=0.;
          vpMeterPixelConversion::convertPoint(fsm.get_cam(),point3D->get_x(),point3D->get_y(),_u,_v);

          int region_width=5;
          int region_height=5;
          int u=(int)_u;
          int v=(int)_v;
          for(int i=std::max(u-region_width,0);
              i<std::min(u+region_width,(int)evt.I.getWidth());
              i++){
            for(int j=std::max(v-region_height,0);
                j<std::min(v+region_height,(int)evt.I.getHeight());
                j++){
              vpDisplay::displayPoint(evt.I,j,i,vpColor::cyan);
            }
          }
          //std::cout << "median:" << boost::accumulators::median(acc) << std::endl;
        }
      }
      vpDisplay::flush(evt.I);

      vpMatrix mat = fsm.get_mbt().getCovarianceMatrix();
      if(fsm.get_cmd().show_plot()){
        if(fsm.get_cmd().using_var_limit())
          plot_->plot(0,6,iter_,(double)fsm.get_cmd().get_var_limit());
        for(int i=0;i<6;i++)
          plot_->plot(0,i,iter_,mat[i][i]);
      }


      iter_++;
    }
  };
}
#endif /* EVENTS_H_ */
