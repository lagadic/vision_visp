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
    /*
     * detect pattern in image
     * image: image where to detect pattern
     * timeout: maximum time for pattern detection
     * offset: offset container box by that much pixels
     * */
    virtual bool detect(cv::Mat& image, int timeout=1000, unsigned int offsetx=0, unsigned int offsety=0) = 0;
    //returns pattern container box as a vector of lines
    std::vector<std::pair<cv::Point,cv::Point> >& get_lines();
    //returns the contained message if there is one
    std::string& get_message();
    //returns pattern container box as a vector of points
    std::vector<cv::Point>& get_polygon();
  };
}
#endif
