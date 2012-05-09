#ifndef __LOGITECH_H__
#define __LOGITECH_H__
#include "cv.h"

class LogitechVideoCapture : public cv::VideoCapture {
  public:
  LogitechVideoCapture(int device) : cv::VideoCapture(device) {
      this->set(CV_CAP_PROP_FRAME_WIDTH,1024);
      this->set(CV_CAP_PROP_FRAME_HEIGHT,768);
  }
};
#endif
