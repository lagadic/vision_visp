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

    struct DetectFlashcodeGeneric : public msm::front::state<>
    {
        virtual vpColor getColor() = 0;
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
