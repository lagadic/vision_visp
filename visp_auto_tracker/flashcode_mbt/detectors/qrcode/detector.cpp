#include "detector.h"

namespace detectors{
namespace qrcode{
  Detector::Detector(){
    // configure the reader
    scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
  }

  bool Detector::detect(cv::Mat& image, int timeout, unsigned int offsetx, unsigned int offsety){
    bool detected = false;
    lines_.clear();
    message_.clear();
    polygon_.clear();

    scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
    int width = image.cols;
    int height = image.rows;

    cv::Mat gray_image;
    cv::cvtColor(image,gray_image,CV_BGR2GRAY);

    // wrap image data
    zbar::Image img(width, height, "Y800", gray_image.data, width * height);

    // scan the image for barcodes
    int n = scanner_.scan(img);

    // extract results
    for(zbar::Image::SymbolIterator symbol = img.symbol_begin();
        symbol != img.symbol_end();
        ++symbol) {
        message_ = symbol->get_data();
        detected = true;

        for(int i=0;
            i<symbol->get_location_size();
            i++
            ){

            polygon_.push_back(cv::Point(symbol->get_location_x(i) + offsetx,symbol->get_location_y(i) + offsety));


        }

        lines_.push_back(std::pair<cv::Point, cv::Point>(cv::Point(polygon_[0].x, polygon_[0].y), cv::Point(polygon_[1].x,
                                                                                                        polygon_[1].y)));
        lines_.push_back(std::pair<cv::Point, cv::Point>(cv::Point(polygon_[1].x, polygon_[1].y), cv::Point(polygon_[2].x,
                                                                                                            polygon_[2].y)));
        lines_.push_back(std::pair<cv::Point, cv::Point>(cv::Point(polygon_[2].x, polygon_[2].y), cv::Point(polygon_[3].x,
                                                                                                            polygon_[3].y)));
        lines_.push_back(std::pair<cv::Point, cv::Point>(cv::Point(polygon_[3].x, polygon_[3].y), cv::Point(polygon_[0].x,
                                                                                                        polygon_[0].y)));


    }

    // clean up
    img.set_data(NULL, 0);

    return detected;
  }
}
}
