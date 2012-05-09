#ifndef __DETECTOR_H__
#define __DETECTOR_H__
#include "cv.h"

#include <vector>
#include <utility>
#include <string>
namespace datamatrix{
  class Detector{
  private:
    std::vector<std::pair<cv::Point,cv::Point> > lines_;
    std::vector<cv::Point> polygon_;
    std::string message_;
  public:
    Detector();
    bool detect(cv::Mat& image, int timeout=1000);
    std::vector<std::pair<cv::Point,cv::Point> >& get_lines();
    std::string& get_message();
    std::vector<cv::Point>& get_polygon();
  };
}
#endif
