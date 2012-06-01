#ifndef __DETECTOR_BASE_H__
#define __DETECTOR_BASE_H__
#include "cv.h"

#include <vector>
#include <utility>
#include <string>

namespace detectors{
  class DetectorBase{
  protected:
    std::vector<std::pair<cv::Point,cv::Point> > lines_;
    std::vector<cv::Point> polygon_;
    std::string message_;
  public:
    virtual bool detect(cv::Mat& image, int timeout=1000, unsigned int offsetx=0, unsigned int offsety=0) = 0;
    std::vector<std::pair<cv::Point,cv::Point> >& get_lines();
    std::string& get_message();
    std::vector<cv::Point>& get_polygon();
  };
}
#endif
