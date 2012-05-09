#include "detector.h"
#include <dmtx.h>

namespace datamatrix{
  Detector::Detector(){
  }

  bool Detector::detect(cv::Mat& image, int timeout){
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
      polygon_.push_back(cv::Point(p00.X,image.rows-p00.Y));
      polygon_.push_back(cv::Point(p10.X,image.rows-p10.Y));
      polygon_.push_back(cv::Point(p11.X,image.rows-p11.Y));
      polygon_.push_back(cv::Point(p01.X,image.rows-p01.Y));

      lines_.push_back(
		       std::pair<cv::Point,cv::Point>(
						      cv::Point(p00.X,image.rows-p00.Y),
						      cv::Point(p10.X,image.rows-p10.Y)
						      )
		       );
      lines_.push_back(
                             std::pair<cv::Point,cv::Point>(
                                                            cv::Point(p10.X,image.rows-p10.Y),
                                                            cv::Point(p11.X,image.rows-p11.Y)
                                                            )
                             );
      lines_.push_back(
                             std::pair<cv::Point,cv::Point>(
                                                            cv::Point(p11.X,image.rows-p11.Y),
                                                            cv::Point(p01.X,image.rows-p01.Y)
                                                            )
                             );
      lines_.push_back(
                             std::pair<cv::Point,cv::Point>(
                                                            cv::Point(p01.X,image.rows-p01.Y),
                                                            cv::Point(p00.X,image.rows-p00.Y)
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

  std::vector<std::pair<cv::Point,cv::Point> >& Detector:: get_lines(){
    return lines_;
  }

  std::string& Detector:: get_message(){
    return message_;
  }

  std::vector<cv::Point>& Detector:: get_polygon(){
    return polygon_;
  }

}
