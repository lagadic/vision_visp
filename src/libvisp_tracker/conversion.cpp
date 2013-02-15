#include <cstring>
#include <stdexcept>

#include <boost/format.hpp>

#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btQuaternion.h>
#include <tf/transform_datatypes.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>

#include <visp/vpImage.h>

#define protected public
# include <visp/vpMbEdgeTracker.h>
#undef protected

#define protected public
# include <visp/vpMbKltTracker.h>
#undef protected

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
      ROS_INFO
	("dst is %dx%d but src size is %dx%d, resizing.",
	 src->width, src->height,
	 dst.getWidth (), dst.getHeight ());
      dst.resize (src->height, src->width);
    }

  if(src->encoding == MONO8)
    memcpy(dst.bitmap,
	   &src->data[0],
	   dst.getHeight () * src->step * sizeof(unsigned char));
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

void vpHomogeneousMatrixToTransform(geometry_msgs::Transform& dst,
				    const vpHomogeneousMatrix& src)
{
  btMatrix3x3 rotation;
  btQuaternion quaternion;
  for(unsigned i = 0; i < 3; ++i)
    for(unsigned j = 0; j < 3; ++j)
      rotation[i][j] = src[i][j];
  rotation.getRotation(quaternion);

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
  btQuaternion quaternion
    (src.rotation.x, src.rotation.y, src.rotation.z, src.rotation.w);
  btMatrix3x3 rotation(quaternion);

  // Copy the rotation component.
  for(unsigned i = 0; i < 3; ++i)
    for(unsigned j = 0; j < 3; ++j)
      dst[i][j] = rotation[i][j];

  // Copy the translation component.
  dst[0][3] = src.translation.x;
  dst[1][3] = src.translation.y;
  dst[2][3] = src.translation.z;
}

void transformToVpHomogeneousMatrix(vpHomogeneousMatrix& dst,
				    const geometry_msgs::Pose& src)
{
  btQuaternion quaternion
    (src.orientation.x, src.orientation.y, src.orientation.z,
     src.orientation.w);
  btMatrix3x3 rotation(quaternion);

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

void convertModelBasedSettingsConfigToVpMe(const visp_tracker::ModelBasedSettingsConfig& config,
				   vpMe& moving_edge,
				   vpMbTracker* tracker)
{
  vpMbEdgeTracker* t = dynamic_cast<vpMbEdgeTracker*>(tracker);
  
  moving_edge.mask_size = config.mask_size;
  moving_edge.n_mask = config.n_mask;
  moving_edge.range = config.range;
  moving_edge.threshold = config.threshold;
  moving_edge.mu1 = config.mu1;
  moving_edge.mu2 = config.mu2;
  moving_edge.sample_step = config.sample_step;
  moving_edge.ntotal_sample = config.ntotal_sample;

  moving_edge.strip = config.strip;
  moving_edge.min_samplestep = config.min_samplestep;
  moving_edge.aberration = config.aberration;
  moving_edge.init_aberration = config.init_aberration;

  t->setLambda(config.lambda);
  t->setFirstThreshold(config.first_threshold);
  
  //FIXME: not sure if this is needed.
  moving_edge.initMask();
  // Reset the tracker and the node state.
  t->setMovingEdge(moving_edge);
}

void convertVpMeToModelBasedSettingsConfig(const vpMe& moving_edge,
				   const vpMbTracker* tracker,
				   visp_tracker::ModelBasedSettingsConfig& config)
{
  const vpMbEdgeTracker* t = dynamic_cast<const vpMbEdgeTracker*>(tracker);
  
  config.mask_size = moving_edge.mask_size;
  config.n_mask = moving_edge.n_mask;
  config.range = moving_edge.range;
  config.threshold = moving_edge.threshold;
  config.mu1 = moving_edge.mu1;
  config.mu2 = moving_edge.mu2;
  config.sample_step = moving_edge.sample_step;
  config.ntotal_sample = moving_edge.ntotal_sample;

  config.strip = moving_edge.strip;
  config.min_samplestep = moving_edge.min_samplestep;
  config.aberration = moving_edge.aberration;
  config.init_aberration = moving_edge.init_aberration;

  config.lambda = t->lambda;
  config.first_threshold = t->percentageGdPt;
}

void convertModelBasedSettingsConfigToVpKltOpencv(const visp_tracker::ModelBasedSettingsConfig& config,
           vpKltOpencv& klt,
           vpMbTracker* tracker)
{
  vpMbKltTracker* t = dynamic_cast<vpMbKltTracker*>(tracker);
  
  klt.setMaxFeatures(config.max_features);
  klt.setWindowSize(config.window_size);
  klt.setQuality(config.quality);
  klt.setMinDistance(config.min_distance);
  klt.setHarrisFreeParameter(config.harris);
  klt.setBlockSize(config.size_block);
  klt.setPyramidLevels(config.pyramid_lvl);
  
  t->setAngleAppear(vpMath::rad(config.angle_appear));
  t->setAngleDisappear(vpMath::rad(config.angle_disappear));
  t->setMaskBorder((unsigned)config.mask_border);
  
  t->setKltOpencv(klt);
}

void convertVpKltOpencvToModelBasedSettingsConfig(const vpKltOpencv& klt,
           const vpMbTracker* tracker,
           visp_tracker::ModelBasedSettingsConfig& config)
{
  const vpMbKltTracker* t = dynamic_cast<const vpMbKltTracker*>(tracker);
  
  config.max_features = klt.getMaxFeatures();
  config.window_size = klt.getWindowSize();
  config.quality = klt.getQuality();
  config.min_distance = klt.getMinDistance();
  config.harris = klt.getHarrisFreeParameter();
  config.size_block = klt.getBlockSize();
  config.pyramid_lvl = klt.getPyramidLevels();
  
  config.angle_appear = vpMath::deg(t->angleAppears);
  config.angle_disappear = vpMath::deg(t->angleDisappears);
  config.mask_border = t->maskBorder;
}

void convertVpMeToInitRequest(const vpMe& moving_edge,
			      const vpMbTracker* tracker,
			      visp_tracker::Init& srv)
{
  const vpMbEdgeTracker* t = dynamic_cast<const vpMbEdgeTracker*>(tracker);
  
  srv.request.moving_edge.mask_size = moving_edge.mask_size;
  srv.request.moving_edge.n_mask = moving_edge.n_mask;
  srv.request.moving_edge.range = moving_edge.range;
  srv.request.moving_edge.threshold = moving_edge.threshold;
  srv.request.moving_edge.mu1 = moving_edge.mu1;
  srv.request.moving_edge.mu2 = moving_edge.mu2;
  srv.request.moving_edge.sample_step = moving_edge.sample_step;
  srv.request.moving_edge.ntotal_sample = moving_edge.ntotal_sample;

  srv.request.moving_edge.strip = moving_edge.strip;
  srv.request.moving_edge.min_samplestep = moving_edge.min_samplestep;
  srv.request.moving_edge.aberration = moving_edge.aberration;
  srv.request.moving_edge.init_aberration = moving_edge.init_aberration;

  srv.request.moving_edge.lambda = t->lambda;
  srv.request.moving_edge.first_threshold = t->percentageGdPt;
}

void convertInitRequestToVpMe(const visp_tracker::Init::Request& req,
			      vpMbTracker* tracker,
			      vpMe& moving_edge)
{
  vpMbEdgeTracker* t = dynamic_cast<vpMbEdgeTracker*>(tracker);
  
  moving_edge.mask_size = req.moving_edge.mask_size;
  moving_edge.n_mask = req.moving_edge.n_mask;
  moving_edge.range = req.moving_edge.range;
  moving_edge.threshold = req.moving_edge.threshold;
  moving_edge.mu1 = req.moving_edge.mu1;
  moving_edge.mu2 = req.moving_edge.mu2;
  moving_edge.sample_step = req.moving_edge.sample_step;
  moving_edge.ntotal_sample = req.moving_edge.ntotal_sample;

  moving_edge.strip = req.moving_edge.strip;
  moving_edge.min_samplestep = req.moving_edge.min_samplestep;
  moving_edge.aberration = req.moving_edge.aberration;
  moving_edge.init_aberration = req.moving_edge.init_aberration;

  t->setLambda(req.moving_edge.lambda);
  t->setFirstThreshold(req.moving_edge.first_threshold);
}

void convertVpKltOpencvToInitRequest(const vpKltOpencv& klt,
            const vpMbTracker* tracker,
            visp_tracker::Init& srv)
{
  const vpMbKltTracker* t = dynamic_cast<const vpMbKltTracker*>(tracker);
  
  srv.request.klt_param.max_features = klt.getMaxFeatures();
  srv.request.klt_param.window_size = klt.getWindowSize();
  srv.request.klt_param.quality = klt.getQuality();
  srv.request.klt_param.min_distance = klt.getMinDistance();
  srv.request.klt_param.harris = klt.getHarrisFreeParameter();
  srv.request.klt_param.size_block = klt.getBlockSize();
  srv.request.klt_param.pyramid_lvl = klt.getPyramidLevels();
  
  srv.request.klt_param.angle_appear = vpMath::deg(t->angleAppears);
  srv.request.klt_param.angle_disappear = vpMath::deg(t->angleDisappears);
  srv.request.klt_param.mask_border = t->maskBorder;
}

void convertInitRequestToVpKltOpencv(const visp_tracker::Init::Request& req,
            vpMbTracker* tracker,
            vpKltOpencv& klt)
{
  vpMbKltTracker* t = dynamic_cast<vpMbKltTracker*>(tracker);
  
  klt.setMaxFeatures(req.klt_param.max_features);
  klt.setWindowSize(req.klt_param.window_size);
  klt.setQuality(req.klt_param.quality);
  klt.setMinDistance(req.klt_param.min_distance);
  klt.setHarrisFreeParameter(req.klt_param.harris);
  klt.setBlockSize(req.klt_param.size_block);
  klt.setPyramidLevels(req.klt_param.pyramid_lvl);
  
  t->setAngleAppear(vpMath::rad(req.klt_param.angle_appear));
  t->setAngleDisappear(vpMath::rad(req.klt_param.angle_disappear));
  t->setMaskBorder((unsigned)req.klt_param.mask_border);
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
