#ifndef __EVENTS_H__
#define __EVENTS_H__
#include "events.h"
#include <cassert>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/core/vpRGBa.h>
#include <visp3/core/vpRect.h>
#include <visp3/gui/vpPlot.h>

namespace msm = boost::msm;
namespace tracking
{
struct WaitingForInput : public msm::front::state<> {
  template <class Event, class Fsm> void on_entry(Event const &, Fsm &fsm)
  {
    if (fsm.get_cmd().get_verbose())
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "entering: WaitingForInput");
  }
  template <class Event, class Fsm> void on_exit(Event const &evt, Fsm &fsm)
  {
    if (fsm.get_cmd().get_verbose())
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "leaving: WaitingForInput");
    if (fsm.get_flush_display()) {
      vpDisplay::display(evt.I);
      vpDisplay::flush(evt.I);
    }
  }
};

struct Finished : public msm::front::state<> {
  template <class Event, class Fsm> void on_entry(Event const & /*evt*/, Fsm &fsm)
  {
    if (fsm.get_cmd().get_verbose()) {
      typename Fsm::statistics_t &statistics = fsm.get_statistics();
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "statistics:");
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "\tglobal:");
      /*       std::vector<int> v{5, 1, 2, 3, 4};

     std::vector<int>::iterator b = statistics.var.begin();
     std::vector<int>::iterator e = statistics.var.end();

     std::vector<int>::iterator med = b;
     std::advance(med, statistics.var.size() / 2);

     // This makes the 2nd position hold the median.
     std::nth_element(b, med, e);
     */
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "\t\tmedian:" << boost::accumulators::median(statistics.var));
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "\t\tmean:" << boost::accumulators::mean(statistics.var));
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "\t\tmax:" << boost::accumulators::max(statistics.var));

      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "\tX:");
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "\t\tmedian:" << boost::accumulators::median(statistics.var_x));
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "\t\tmean:" << boost::accumulators::mean(statistics.var_x));
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "\t\tmax:" << boost::accumulators::max(statistics.var_x));

      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "\tY:");
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "\t\tmedian:" << boost::accumulators::median(statistics.var_y));
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "\t\tmean:" << boost::accumulators::mean(statistics.var_y));
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "\t\tmax:" << boost::accumulators::max(statistics.var_y));

      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "\tZ:");
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "\t\tmedian:" << boost::accumulators::median(statistics.var_z));
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "\t\tmean:" << boost::accumulators::mean(statistics.var_z));
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "\t\tmax:" << boost::accumulators::max(statistics.var_z));

      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "\tW_X:");
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "\t\tmedian:" << boost::accumulators::median(statistics.var_wx));
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "\t\tmean:" << boost::accumulators::mean(statistics.var_wx));
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "\t\tmax:" << boost::accumulators::max(statistics.var_wx));

      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "\tW_Y:");
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "\t\tmedian:" << boost::accumulators::median(statistics.var_wy));
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "\t\tmean:" << boost::accumulators::mean(statistics.var_wy));
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "\t\tmax:" << boost::accumulators::max(statistics.var_wy));

      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "\tW_Z:");
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "\t\tmedian:" << boost::accumulators::median(statistics.var_wz));
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "\t\tmean:" << boost::accumulators::mean(statistics.var_wz));
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "\t\tmax:" << boost::accumulators::max(statistics.var_wz));
    }
  }
};

struct DetectFlashcodeGeneric : public msm::front::state<> {
  vpImagePoint corner0;
  vpImagePoint corner1;
  vpImagePoint corner2;
  vpImagePoint corner3;
  virtual vpColor getColor() = 0;
  template <class Fsm> void on_entry(finished const &evt, Fsm &fsm) {}

  template <class Fsm> void on_exit(finished const & /*evt*/, Fsm & /*fsm*/) {}

  template <class Event, class Fsm> void on_entry(Event const &, Fsm &fsm)
  {
    if (fsm.get_cmd().get_verbose())
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "entering: DetectFlashcode");
  }
  template <class Event, class Fsm> void on_exit(Event const &evt, Fsm &fsm)
  {
    if (fsm.get_cmd().get_verbose())
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "leaving: DetectFlashcode");
    if (fsm.get_flush_display()) {
      vpDisplay::display(evt.I);
    }

    // TODO: add a parameter to be able to select the QRcode from it's message
    // For the moment we get the position of the first code that is the largest in the image
    std::vector<std::vector<vpImagePoint> > polygons = fsm.get_detector().getPolygon();
    std::vector<vpImagePoint> polygon(4);
    if (polygons.size())
      polygon = polygons[0];
    if (polygon.size() != 4) {
      if (fsm.get_flush_display())
        vpDisplay::flush(evt.I);
      return;
    }
    corner0 = polygon[0];
    corner1 = polygon[1];
    corner2 = polygon[2];
    corner3 = polygon[3];

    vpDisplay::flush(evt.I);
  }
};

struct DetectFlashcode : public DetectFlashcodeGeneric {
  vpColor getColor() { return vpColor::green; }
};
struct ReDetectFlashcode : public DetectFlashcodeGeneric {
  template <class Event, class Fsm> void on_entry(Event const &, Fsm &fsm)
  {
    if (fsm.get_cmd().get_verbose())
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "entering: ReDetectFlashcode");
  }
  vpColor getColor() { return vpColor::orange; }
};

struct DetectModel : public msm::front::state<> {
  std::vector<vpImagePoint> model_inner_corner;
  std::vector<vpImagePoint> model_outer_corner;
  vpHomogeneousMatrix cMo;

  DetectModel() : model_inner_corner(4), model_outer_corner(4) {}
  template <class Fsm> void on_entry(finished const &evt, Fsm &fsm) {}

  template <class Fsm> void on_exit(finished const & /*evt*/, Fsm & /*fsm*/) {}

  template <class Event, class Fsm> void on_entry(Event const &, Fsm &fsm)
  {
    if (fsm.get_cmd().get_verbose())
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "entering: DetectModel");
  }
  template <class Event, class Fsm> void on_exit(Event const & /*evt*/, Fsm &fsm)
  {
    if (fsm.get_cmd().get_verbose())
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "leaving: DetectModel");
    std::vector<vpPoint> &points3D_inner = fsm.get_points3D_inner();
    std::vector<vpPoint> &points3D_outer = fsm.get_points3D_outer();

    fsm.get_mbt().getPose(cMo);

    for (unsigned int i = 0; i < 4; i++) {
      vpMeterPixelConversion::convertPoint(fsm.get_cam(), points3D_outer[i].get_x(), points3D_outer[i].get_y(),
                                           model_outer_corner[i]);
      vpMeterPixelConversion::convertPoint(fsm.get_cam(), points3D_inner[i].get_x(), points3D_inner[i].get_y(),
                                           model_inner_corner[i]);
    }
    if (fsm.get_flush_display()) {
      vpImage<vpRGBa> &I = fsm.get_I();
      vpDisplay::displayText(I, model_inner_corner[0], std::string("mi1"), vpColor::blue);
      vpDisplay::displayCross(I, model_inner_corner[0], 2, vpColor::blue, 2);
      vpDisplay::displayText(I, model_inner_corner[1], std::string("mi2"), vpColor::yellow);
      vpDisplay::displayCross(I, model_inner_corner[1], 2, vpColor::yellow, 2);
      vpDisplay::displayText(I, model_inner_corner[2], std::string("mi3"), vpColor::cyan);
      vpDisplay::displayCross(I, model_inner_corner[2], 2, vpColor::cyan, 2);
      vpDisplay::displayText(I, model_inner_corner[3], std::string("mi4"), vpColor::darkRed);
      vpDisplay::displayCross(I, model_inner_corner[3], 2, vpColor::darkRed, 2);

      vpDisplay::displayText(I, model_outer_corner[0], std::string("mo1"), vpColor::blue);
      vpDisplay::displayCross(I, model_outer_corner[0], 2, vpColor::blue, 2);
      vpDisplay::displayText(I, model_outer_corner[1], std::string("mo2"), vpColor::yellow);
      vpDisplay::displayCross(I, model_outer_corner[1], 2, vpColor::yellow, 2);
      vpDisplay::displayText(I, model_outer_corner[2], std::string("mo3"), vpColor::cyan);
      vpDisplay::displayCross(I, model_outer_corner[2], 2, vpColor::cyan, 2);
      vpDisplay::displayText(I, model_outer_corner[3], std::string("mo4"), vpColor::darkRed);
      vpDisplay::displayCross(I, model_outer_corner[3], 2, vpColor::darkRed, 2);

      try {
        fsm.get_mbt().display(I, cMo, fsm.get_cam(), vpColor::blue, 1); // display the model at the computed pose.
      } catch (vpException &e) {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Cannot display the model");
      }

      vpDisplay::flush(I);
    }
  }
};

class TrackModel : public msm::front::state<>
{
private:
  vpPlot *plot_;
  int iter_;

public:
  vpHomogeneousMatrix cMo;
  vpMatrix covariance;

  ~TrackModel() { delete plot_; }

  TrackModel() : plot_(NULL), iter_(0) {}

  template <class Fsm> void on_entry(finished const &evt, Fsm &fsm)
  {
    fsm.get_mbt().getPose(cMo);
    covariance = fsm.get_mbt().getCovarianceMatrix();
  }

  template <class Fsm> void on_exit(finished const & /*evt*/, Fsm & /*fsm*/) {}

  template <class Event, class Fsm> void on_entry(Event const & /*evt*/, Fsm &fsm)
  {
    if (fsm.get_cmd().show_plot() && plot_ == NULL) {
      plot_ = new vpPlot(1, 700, 700, 100, 200, "Variances");
      plot_->initGraph(0, 7);
    }
  }
  template <class Event, class Fsm> void on_exit(Event const &evt, Fsm &fsm)
  {
    fsm.get_mbt().getPose(cMo);
    covariance = fsm.get_mbt().getCovarianceMatrix();
    if (fsm.get_flush_display()) {
      vpDisplay::display(evt.I);

      fsm.get_mbt().display(evt.I, cMo, fsm.get_cam(), vpColor::red, 1); // display the model at the computed pose.
      vpDisplay::displayFrame(evt.I, cMo, fsm.get_cam(), .1, vpColor::none, 2);
      if (fsm.get_cmd().using_adhoc_recovery() && fsm.get_cmd().get_adhoc_recovery_display()) {
        for (unsigned int p = 0; p < fsm.get_points3D_middle().size(); p++) {
          vpPoint &point3D = fsm.get_points3D_middle()[p];
          vpPoint &point3D_inner = fsm.get_points3D_inner()[p];
          double _u = 0., _v = 0., _u_inner = 0., _v_inner = 0.;

          vpMeterPixelConversion::convertPoint(fsm.get_cam(), point3D.get_x(), point3D.get_y(), _u, _v);
          vpMeterPixelConversion::convertPoint(fsm.get_cam(), point3D_inner.get_x(), point3D_inner.get_y(), _u_inner,
                                               _v_inner);
          int region_width = std::max((int)(std::abs(_u - _u_inner) * fsm.get_cmd().get_adhoc_recovery_size()), 1);
          int region_height = std::max((int)(std::abs(_v - _v_inner) * fsm.get_cmd().get_adhoc_recovery_size()), 1);

          int u = (int)_u;
          int v = (int)_v;
          vpDisplay::displayRectangle(evt.I,
                                      vpImagePoint(std::max(v - region_height, 0), std::max(u - region_width, 0)),
                                      vpImagePoint(std::max(0, std::min(v + region_height, (int)evt.I.getHeight())),
                                                   std::max(0, std::min(u + region_width, (int)evt.I.getWidth()))),
                                      vpColor::cyan, true);
        }
      }
      vpDisplay::flush(evt.I);

      if (fsm.get_cmd().show_plot()) {
        if (fsm.get_cmd().using_var_limit())
          plot_->plot(0, 6, iter_, (double)fsm.get_cmd().get_var_limit());
        for (unsigned int i = 0; i < 6; i++)
          plot_->plot(0, i, iter_, covariance[i][i]);
      }
    }

    iter_++;
  }
};
} // namespace tracking
#endif /* EVENTS_H_ */
