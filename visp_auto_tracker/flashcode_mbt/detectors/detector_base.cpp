#include "detector_base.h"

namespace detectors{
std::vector<std::pair<cv::Point,cv::Point> >& DetectorBase:: get_lines(){
  return lines_;
}

std::string& DetectorBase:: get_message(){
  return message_;
}

std::vector<cv::Point>& DetectorBase:: get_polygon(){
  return polygon_;
}
}


