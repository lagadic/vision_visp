#ifndef __QRDETECTOR_H__
#define __QRDETECTOR_H__
#include "cv.h"

#include <vector>
#include <utility>
#include <string>
#include "detectors/detector_base.h"
#include <zbar.h>

namespace detectors{
namespace qrcode{
  class Detector : public DetectorBase{
  private:
    zbar::ImageScanner scanner_;
  public:
    Detector();
    bool detect(cv::Mat& image, int timeout=1000, unsigned int offsetx = 0, unsigned int offsety = 0);
  };
}
}
#endif
