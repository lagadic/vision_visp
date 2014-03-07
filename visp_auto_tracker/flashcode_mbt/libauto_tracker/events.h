#ifndef __TEVENTS_H__
#define __TEVENTS_H__
#include <visp/vpImage.h>
#include <visp/vpCameraParameters.h>

namespace tracking{

  struct input_ready{
    input_ready(vpImage<vpRGBa>& I,vpCameraParameters& cam) : I(I),cam_(cam),frame(0){}
    input_ready(vpImage<vpRGBa>& I,vpCameraParameters& cam,int frame) : I(I),cam_(cam),frame(frame){}
    vpImage<vpRGBa>& I;
    vpCameraParameters cam_;
    int frame;
  };
  struct select_input{
    select_input(vpImage<vpRGBa>& I) : I(I){}
    vpImage<vpRGBa>& I;
  };
  struct finished{
  };
}
#endif
