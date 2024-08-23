#include <cstring>
#include <stdexcept>

#include <boost/format.hpp>

#include <tf/transform_datatypes.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>

#include <visp3/core/vpImage.h>
#include <visp3/core/vpTranslationVector.h>
#include <visp3/core/vpQuaternionVector.h>

#include <visp3/mbt/vpMbGenericTracker.h>


#include "conversion.hh"

void rosImageToVisp(vpImage<unsigned char>& dst,
                    const sensor_msgs::Image::ConstPtr& src)
{
  using sensor_msgs::image_encodings::RGB8;
  using sensor_msgs::image_encodings::RGBA8;
  using sensor_msgs::image_encodings::BGR8;
  using sensor_msgs::image_encodings::BGRA8;
  using sensor_msgs::image_encodings::MONO8;
  using sensor_msgs::image_encodings::MONO16;
  using sensor_msgs::image_encodings::numChannels;

  // Resize the image if necessary.
  if (src->width != dst.getWidth() || src->height != dst.getHeight())
  {
    ROS_INFO("dst is %dx%d but src size is %dx%d, resizing.",
             dst.getWidth (), dst.getHeight (), src->width, src->height);
    dst.resize (src->height, src->width);
  }

  if(src->encoding == MONO8) {
    memcpy(dst.bitmap, &src->data[0], dst.getHeight () * src->step * sizeof(unsigned char));
  }
  else if(src->encoding == RGB8 || src->encoding == RGBA8
          || src->encoding == BGR8 || src->encoding == BGRA8)
  {
    unsigned nc = numChannels(src->encoding);
    unsigned cEnd =
        (src->encoding == RGBA8 || src->encoding == BGRA8) ? nc - 1 : nc;

    for(unsigned i = 0; i < dst.getWidth (); ++i)
      for(unsigned j = 0; j < dst.getHeight (); ++j)
      {
        int acc = 0;
        for(unsigned c = 0; c < cEnd; ++c)
          acc += src->data[j * src->step + i * nc + c];
        dst[j][i] =  acc / nc;
      }
  }
  else
  {
    boost::format fmt("bad encoding '%1'");
    fmt % src->encoding;
    throw std::runtime_error(fmt.str());
  }
}

void vispImageToRos(sensor_msgs::Image& dst,
                    const vpImage<unsigned char>& src)
{
  dst.width = src.getWidth();
  dst.height = src.getHeight();
  dst.encoding = sensor_msgs::image_encodings::MONO8;
  dst.step = src.getWidth();
  dst.data.resize(dst.height * dst.step);
  for(unsigned i = 0; i < src.getWidth (); ++i)
    for(unsigned j = 0; j < src.getHeight (); ++j)
      dst.data[j * dst.step + i] = src[j][i];
}


std::string convertVpMbTrackerToRosMessage(const vpMbGenericTracker &tracker)
{
  std::stringstream stream;
  stream << "Model Based Tracker Common Setttings\n" <<
            " Angle for polygons apparition...." << vpMath::deg(tracker.getAngleAppear()) <<" degrees\n" <<
            " Angle for polygons disparition..." << vpMath::deg(tracker.getAngleDisappear()) << " degrees\n";
  return stream.str();
}

std::string convertVpMeToRosMessage(const vpMbGenericTracker &tracker, const vpMe& moving_edge)
{
  std::stringstream stream;
  stream  << "Moving Edge Setttings\n" <<
             " Size of the convolution masks...." << moving_edge.getMaskSize() <<"x"<< moving_edge.getMaskSize() <<" pixels\n" <<
             " Query range +/- J................" << moving_edge.getRange() <<" pixels\n" <<
             " Likelihood test ratio............" << moving_edge.getThreshold() << "\n" <<
             " Contrast tolerance +/-..........." << moving_edge.getMu1() * 100 << "% and " << moving_edge.getMu2() * 100 << "% \n" <<
             " Sample step......................" << moving_edge.getSampleStep() <<" pixels\n" <<
             " Strip............................" << moving_edge.getStrip() << " pixels\n";

  stream  << " Good moving edge threshold......." << tracker.getGoodMovingEdgesRatioThreshold()*100 << "%\n";

  return stream.str();
}

std::string convertVpKltOpencvToRosMessage(const vpMbGenericTracker &tracker, const vpKltOpencv& klt)
{
  std::stringstream stream;
  stream << "KLT Setttings\n" <<
            " Window size......................" << klt.getWindowSize() <<"x"<< klt.getWindowSize() <<" pixels\n" <<
            " Mask border......................" << tracker.getKltMaskBorder() << " pixels\n" <<
            " Maximum number of features......." << klt.getMaxFeatures() <<"\n" <<
            " Detected points quality.........." << klt.getQuality() << "\n" <<
            " Minimum distance between points.." << klt.getMinDistance() << " pixels\n" <<
            " Harris free parameter............" << klt.getHarrisFreeParameter() <<"\n" <<
            " Block size......................." << klt.getBlockSize() << "x" << klt.getBlockSize() << " pixels\n" <<
            " Number of pyramid levels........." << klt.getPyramidLevels() << "\n";

  return stream.str();
}

void vpHomogeneousMatrixToTransform(geometry_msgs::Transform& dst,
                                    const vpHomogeneousMatrix& src)
{
  vpQuaternionVector quaternion;
  src.extract(quaternion);

  dst.translation.x = src[0][3];
  dst.translation.y = src[1][3];
  dst.translation.z = src[2][3];

  dst.rotation.x = quaternion.x();
  dst.rotation.y = quaternion.y();
  dst.rotation.z = quaternion.z();
  dst.rotation.w = quaternion.w();
}

void transformToVpHomogeneousMatrix(vpHomogeneousMatrix& dst,
                                    const geometry_msgs::Transform& src)
{
  vpTranslationVector translation(src.translation.x,src.translation.y,src.translation.z);
  vpQuaternionVector quaternion(src.rotation.x,src.rotation.y,src.rotation.z,src.rotation.w);
#if VISP_VERSION_INT > VP_VERSION_INT(3,6,0)
  dst.build(translation, quaternion);
#else
  dst.buildFrom(translation, quaternion);
#endif
}

void transformToVpHomogeneousMatrix(vpHomogeneousMatrix& dst,
                                    const geometry_msgs::Pose& src)
{
  vpQuaternionVector quaternion
      (src.orientation.x, src.orientation.y, src.orientation.z,
       src.orientation.w);
  vpRotationMatrix rotation(quaternion);

  // Copy the rotation component.
  for(unsigned i = 0; i < 3; ++i)
    for(unsigned j = 0; j < 3; ++j)
      dst[i][j] = rotation[i][j];

  // Copy the translation component.
  dst[0][3] = src.position.x;
  dst[1][3] = src.position.y;
  dst[2][3] = src.position.z;
}

void transformToVpHomogeneousMatrix(vpHomogeneousMatrix& dst,
                                    const tf::Transform& src)
{
  // Copy the rotation component.
  for(unsigned i = 0; i < 3; ++i)
    for(unsigned j = 0; j < 3; ++j)
      dst[i][j] = src.getBasis ()[i][j];

  // Copy the translation component.
  for (unsigned i = 0; i < 3; ++i)
    dst[i][3] = src.getOrigin ()[i];
  dst[3][3] = 1.;
}

void convertVpMbTrackerToInitRequest(const vpMbGenericTracker &tracker,
                                     visp_tracker::Init& srv)
{
  srv.request.tracker_param.angle_appear = vpMath::deg(tracker.getAngleAppear());
  srv.request.tracker_param.angle_disappear = vpMath::deg(tracker.getAngleDisappear());
}

void convertInitRequestToVpMbTracker(const visp_tracker::Init::Request& req,
                                     vpMbGenericTracker &tracker)
{
  tracker.setAngleAppear(vpMath::rad(req.tracker_param.angle_appear));
  tracker.setAngleDisappear(vpMath::rad(req.tracker_param.angle_disappear));
}

void convertVpMeToInitRequest(const vpMe& moving_edge,
                              const vpMbGenericTracker &tracker,
                              visp_tracker::Init& srv)
{
  srv.request.moving_edge.first_threshold = tracker.getGoodMovingEdgesRatioThreshold();
  srv.request.moving_edge.mask_size = moving_edge.getMaskSize();
  srv.request.moving_edge.range = moving_edge.getRange();
  srv.request.moving_edge.threshold = moving_edge.getThreshold();
  srv.request.moving_edge.mu1 = moving_edge.getMu1();
  srv.request.moving_edge.mu2 = moving_edge.getMu2();
  srv.request.moving_edge.sample_step = moving_edge.getSampleStep();
  srv.request.moving_edge.strip = moving_edge.getStrip();
}

void convertInitRequestToVpMe(const visp_tracker::Init::Request& req,
                              vpMbGenericTracker &tracker,
                              vpMe& moving_edge)
{
  tracker.setGoodMovingEdgesRatioThreshold(req.moving_edge.first_threshold);
  moving_edge.setMaskSize( req.moving_edge.mask_size );
  moving_edge.setRange( req.moving_edge.range );
  moving_edge.setThreshold( req.moving_edge.threshold );
  moving_edge.setMu1( req.moving_edge.mu1 );
  moving_edge.setMu2( req.moving_edge.mu2 );
  moving_edge.setSampleStep( req.moving_edge.sample_step );
  moving_edge.setStrip( req.moving_edge.strip );

  //FIXME: not sure if this is needed.
  moving_edge.initMask();
  //Reset the tracker and the node state.
  tracker.setMovingEdge(moving_edge);
}

void convertVpKltOpencvToInitRequest(const vpKltOpencv& klt,
                                     const vpMbGenericTracker &tracker,
                                     visp_tracker::Init& srv)
{  
  srv.request.klt_param.max_features = klt.getMaxFeatures();
  srv.request.klt_param.window_size = klt.getWindowSize();
  srv.request.klt_param.quality = klt.getQuality();
  srv.request.klt_param.min_distance = klt.getMinDistance();
  srv.request.klt_param.harris = klt.getHarrisFreeParameter();
  srv.request.klt_param.size_block = klt.getBlockSize();
  srv.request.klt_param.pyramid_lvl = klt.getPyramidLevels();
  srv.request.klt_param.mask_border = tracker.getKltMaskBorder();
}

void convertInitRequestToVpKltOpencv(const visp_tracker::Init::Request& req,
                                     vpMbGenericTracker &tracker,
                                     vpKltOpencv& klt)
{  
  klt.setMaxFeatures(req.klt_param.max_features);
  klt.setWindowSize(req.klt_param.window_size);
  klt.setQuality(req.klt_param.quality);
  klt.setMinDistance(req.klt_param.min_distance);
  klt.setHarrisFreeParameter(req.klt_param.harris);
  klt.setBlockSize(req.klt_param.size_block);
  klt.setPyramidLevels(req.klt_param.pyramid_lvl);
  tracker.setKltMaskBorder((unsigned)req.klt_param.mask_border);

  tracker.setKltOpencv(klt);
}

void initializeVpCameraFromCameraInfo(vpCameraParameters& cam,
                                      sensor_msgs::CameraInfoConstPtr info)
{
  if (!info)
    throw std::runtime_error ("missing camera calibration data");

  // Check that the camera is calibrated, as specified in the
  // sensor_msgs/CameraInfo message documentation.
  if (info->K.size() != 3 * 3 || info->K[0] == 0.)
    throw std::runtime_error ("uncalibrated camera");

  // Check matrix size.
  if (!info || info->P.size() != 3 * 4)
    throw std::runtime_error
      ("camera calibration P matrix has an incorrect size");

  if (info->distortion_model.empty ())
  {
    const double& px = info->K[0 * 3 + 0];
    const double& py = info->K[1 * 3 + 1];
    const double& u0 = info->K[0 * 3 + 2];
    const double& v0 = info->K[1 * 3 + 2];
    cam.initPersProjWithoutDistortion(px, py, u0, v0);
    return;
  }

  if (info->distortion_model == sensor_msgs::distortion_models::PLUMB_BOB)
  {
    const double& px = info->P[0 * 4 + 0];
    const double& py = info->P[1 * 4 + 1];
    const double& u0 = info->P[0 * 4 + 2];
    const double& v0 = info->P[1 * 4 + 2];
    cam.initPersProjWithoutDistortion(px, py, u0, v0);
    return;
  }

  throw std::runtime_error ("unsupported distortion model");
}
