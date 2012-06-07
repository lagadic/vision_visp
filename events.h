#ifndef __TEVENTS_H__
#define __TEVENTS_H__
#include <visp/vpImage.h>

namespace tracking{

  struct input_ready{
    input_ready(vpImage<vpRGBa>& I) : I(I){}
    vpImage<vpRGBa>& I;
  };
  struct select_input{
    select_input(vpImage<vpRGBa>& I) : I(I){}
    vpImage<vpRGBa>& I;
  };
  struct finished{
  };
}
#endif
