#include "detector.h"
#include <dmtx.h>

namespace detectors{
namespace datamatrix{
  Detector::Detector(){
  }

  bool Detector::detect(cv::Mat& image, int timeout, unsigned int offsetx, unsigned int offsety){
    bool detected = false;
    lines_.clear();
    message_.clear();
    polygon_.clear();
    DmtxRegion     *reg;
    DmtxDecode     *dec;
    DmtxImage      *img;
    DmtxMessage    *msg;
    DmtxTime       t;

    img = dmtxImageCreate(image.data, image.cols, image.rows, DmtxPack24bppRGB);
    //dmtxImageSetProp(img, DmtxPropImageFlip, DmtxFlipY);

    dec = dmtxDecodeCreate(img, 1);
    assert(dec != NULL);

    t = dmtxTimeAdd(dmtxTimeNow(), timeout);
    reg = dmtxRegionFindNext(dec, &t);

    if(reg != NULL) {

      int height;
      int dataWordLength;
      int rotateInt;
      double rotate;
      DmtxVector2 p00, p10, p11, p01;

      height = dmtxDecodeGetProp(dec, DmtxPropHeight);

      p00.X = p00.Y = p10.Y = p01.X = 0.0;
      p10.X = p01.Y = p11.X = p11.Y = 1.0;
      dmtxMatrix3VMultiplyBy(&p00, reg->fit2raw);
      dmtxMatrix3VMultiplyBy(&p10, reg->fit2raw);
      dmtxMatrix3VMultiplyBy(&p11, reg->fit2raw);
      dmtxMatrix3VMultiplyBy(&p01, reg->fit2raw);
      polygon_.push_back(cv::Point(p00.X + offsetx,image.rows-p00.Y + offsety));
      polygon_.push_back(cv::Point(p10.X + offsetx,image.rows-p10.Y + offsety));
      polygon_.push_back(cv::Point(p11.X + offsetx,image.rows-p11.Y + offsety));
      polygon_.push_back(cv::Point(p01.X + offsetx,image.rows-p01.Y + offsety));

      lines_.push_back(
                       std::pair<cv::Point,cv::Point>(
                                                      cv::Point(p00.X + offsetx,image.rows-p00.Y + offsety),
                                                      cv::Point(p10.X + offsetx,image.rows-p10.Y + offsety)
                                                      )
                       );
      lines_.push_back(
                             std::pair<cv::Point,cv::Point>(
                                                            cv::Point(p10.X + offsetx,image.rows-p10.Y + offsety),
                                                            cv::Point(p11.X + offsetx,image.rows-p11.Y + offsety)
                                                            )
                             );
      lines_.push_back(
                             std::pair<cv::Point,cv::Point>(
                                                            cv::Point(p11.X + offsetx,image.rows-p11.Y + offsety),
                                                            cv::Point(p01.X + offsetx,image.rows-p01.Y + offsety)
                                                            )
                             );
      lines_.push_back(
                             std::pair<cv::Point,cv::Point>(
                                                            cv::Point(p01.X + offsetx,image.rows-p01.Y + offsety),
                                                            cv::Point(p00.X + offsetx,image.rows-p00.Y + offsety)
                                                            )
                             );
      detected = true;
      msg = dmtxDecodeMatrixRegion(dec, reg, DmtxUndefined);
      if(msg != NULL) {
        message_ = (const char*)msg->output;
        dmtxMessageDestroy(&msg);
      }
      dmtxRegionDestroy(&reg);
    }
      
    dmtxDecodeDestroy(&dec);
    dmtxImageDestroy(&img);
    return detected;
  }
}
}
