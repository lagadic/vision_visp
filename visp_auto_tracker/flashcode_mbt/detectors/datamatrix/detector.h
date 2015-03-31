#ifndef __DMDETECTOR_H__
#define __DMDETECTOR_H__
#include "cv.h"

#include <vector>
#include <utility>
#include <string>

#include "detectors/detector_base.h"
namespace detectors{
namespace datamatrix{
  class Detector : public DetectorBase{
  public:
    Detector();
    bool detect(cv::Mat& image, int timeout=1000, unsigned int offsetx = 0, unsigned int offsety = 0);
  };
}
}
#endif
