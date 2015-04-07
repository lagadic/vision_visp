#ifndef VISP_TRACKER_CONVERSION_HH
# define VISP_TRACKER_CONVERSION_HH
# include <boost/optional.hpp>

# include <ros/ros.h>

# include <geometry_msgs/Transform.h>
# include <sensor_msgs/Image.h>
# include <sensor_msgs/CameraInfo.h>
# include <tf/transform_datatypes.h>

# include <visp_tracker/Init.h>

#include <visp/vpConfig.h>
#if VISP_VERSION_INT < VP_VERSION_INT(2,10,0)
# define protected public
#endif
# include <visp/vpMbEdgeTracker.h>
# include <visp/vpMbKltTracker.h>
# include <visp/vpMbTracker.h>
#if VISP_VERSION_INT < VP_VERSION_INT(2,10,0)
# undef protected
#endif

# include <visp/vpHomogeneousMatrix.h>
# include <visp/vpCameraParameters.h>
# include <visp/vpMe.h>
# include <visp/vpKltOpencv.h>

/// \brief Convert a ROS image into a ViSP one.
///
/// This function copy a ROS image into a ViSP image.
/// If the size are not matching, the ViSP image will be
/// resized.
///
/// \warning Some encodings only are supported.
///
/// \param dst ViSP destination image
/// \param src ROS source image
void rosImageToVisp(vpImage<unsigned char>& dst,
		    const sensor_msgs::Image::ConstPtr& src);

/// \brief Convert a ViSP image into a ROS one.
///
/// This function copy a ViSP image into a ROS image.
/// The whole content of the ROS image will be reset except
/// the following field which will not be set:
/// - header
/// - is_bigendian
///
/// \param dst ROS destination image
/// \param src ViSP source image
void vispImageToRos(sensor_msgs::Image& dst,
		    const vpImage<unsigned char>& src);

std::string convertVpMbTrackerToRosMessage(const vpMbTracker* tracker);

std::string convertVpMeToRosMessage(const vpMbTracker* tracker, const vpMe& moving_edge);

std::string convertVpKltOpencvToRosMessage(const vpMbTracker* tracker, const vpKltOpencv& klt);

void vpHomogeneousMatrixToTransform(geometry_msgs::Transform& dst,
				    const vpHomogeneousMatrix& src);

void transformToVpHomogeneousMatrix(vpHomogeneousMatrix& dst,
				    const geometry_msgs::Transform& src);

void transformToVpHomogeneousMatrix(vpHomogeneousMatrix& dst,
				    const tf::Transform& src);

void transformToVpHomogeneousMatrix(vpHomogeneousMatrix& dst,
				    const geometry_msgs::Pose& src);

void convertVpMbTrackerToInitRequest(const vpMbTracker* tracker,
            visp_tracker::Init& srv);

void convertInitRequestToVpMbTracker(const visp_tracker::Init::Request& req,
            vpMbTracker* tracker);

void convertVpMeToInitRequest(const vpMe& moving_edge,
			      const vpMbTracker* tracker,
			      visp_tracker::Init& srv);

void convertInitRequestToVpMe(const visp_tracker::Init::Request& req,
			      vpMbTracker* tracker,
			      vpMe& moving_edge);

void convertVpKltOpencvToInitRequest(const vpKltOpencv& klt,
            const vpMbTracker* tracker,
            visp_tracker::Init& srv);

void convertInitRequestToVpKltOpencv(const visp_tracker::Init::Request& req,
            vpMbTracker* tracker,
            vpKltOpencv& klt);

void initializeVpCameraFromCameraInfo(vpCameraParameters& cam,
              sensor_msgs::CameraInfoConstPtr info);

// Dynamic reconfigure template functions
template<class ConfigType>
void convertModelBasedSettingsConfigToVpMbTracker(const ConfigType& config,
                                                  vpMbTracker* tracker)
{
#if VISP_VERSION_INT >= VP_VERSION_INT(2,10,0)
  tracker->setAngleAppear(vpMath::rad(config.angle_appear));
  tracker->setAngleDisappear(vpMath::rad(config.angle_disappear));
#else
  vpMbEdgeTracker* tracker_edge = dynamic_cast<vpMbEdgeTracker*>(tracker);
  if (tracker_edge != NULL) { // Also valid when hybrid
    ROS_INFO("Set param angle from edge");
    tracker_edge->setAngleAppear(vpMath::rad(config.angle_appear));
    tracker_edge->setAngleDisappear(vpMath::rad(config.angle_disappear));
  }
  else {
    vpMbKltTracker* tracker_klt = dynamic_cast<vpMbKltTracker*>(tracker);
    if (tracker_klt != NULL) {
      ROS_INFO("Set param angle from klt");
      tracker_klt->setAngleAppear(vpMath::rad(config.angle_appear));
      tracker_klt->setAngleDisappear(vpMath::rad(config.angle_disappear));
    }
  }
#endif
}

template<class ConfigType>
void convertVpMbTrackerToModelBasedSettingsConfig(const vpMbTracker* tracker,
                                                  ConfigType& config)
{
#if VISP_VERSION_INT >= VP_VERSION_INT(2,10,0)
  config.angle_appear = vpMath::deg(tracker->getAngleAppear());
  config.angle_disappear = vpMath::deg(tracker->getAngleDisappear());
#else
  const vpMbEdgeTracker* tracker_edge = dynamic_cast<const vpMbEdgeTracker*>(tracker);
  if (tracker_edge != NULL) {
    ROS_INFO("Modif config param angle from edge");
    config.angle_appear = vpMath::deg(tracker_edge->getAngleAppear());
    config.angle_disappear = vpMath::deg(tracker_edge->getAngleDisappear());
  }
  else {
    const vpMbKltTracker* tracker_klt = dynamic_cast<const vpMbKltTracker*>(tracker);
    if (tracker_klt != NULL) {
      ROS_INFO("Modif config param angle from klt");
      config.angle_appear = vpMath::deg(tracker_klt->getAngleAppear());
      config.angle_disappear = vpMath::deg(tracker_klt->getAngleDisappear());
    }
  }
#endif
}

template<class ConfigType>
void convertModelBasedSettingsConfigToVpMe(const ConfigType& config,
           vpMe& moving_edge,
           vpMbTracker* tracker)
{
  vpMbEdgeTracker* t = dynamic_cast<vpMbEdgeTracker*>(tracker);

  moving_edge.mask_size = config.mask_size;
  moving_edge.range = config.range;
  moving_edge.threshold = config.threshold;
  moving_edge.mu1 = config.mu1;
  moving_edge.mu2 = config.mu2;
  moving_edge.sample_step = config.sample_step;
  moving_edge.strip = config.strip;

#if VISP_VERSION_INT >= VP_VERSION_INT(2,10,0)
  t->setGoodMovingEdgesRatioThreshold(config.first_threshold);

#else
  t->setFirstThreshold(config.first_threshold);
#endif

  //FIXME: not sure if this is needed.
  moving_edge.initMask();
  //Reset the tracker and the node state.
  t->setMovingEdge(moving_edge);
}

template<class ConfigType>
void convertVpMeToModelBasedSettingsConfig(const vpMe& moving_edge,
           const vpMbTracker* tracker,
           ConfigType& config)
{
  const vpMbEdgeTracker* t = dynamic_cast<const vpMbEdgeTracker*>(tracker);

  config.mask_size = moving_edge.mask_size;
  config.range = moving_edge.range;
  config.threshold = moving_edge.threshold;
  config.mu1 = moving_edge.mu1;
  config.mu2 = moving_edge.mu2;
  config.sample_step = moving_edge.sample_step;
  config.strip = moving_edge.strip;

#if VISP_VERSION_INT >= VP_VERSION_INT(2,10,0)
  config.first_threshold = t->getGoodMovingEdgesRatioThreshold();
#else
  config.first_threshold = t->getFirstThreshold();
#endif
}

template<class ConfigType>
void convertModelBasedSettingsConfigToVpKltOpencv(const ConfigType& config,
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
  t->setMaskBorder((unsigned)config.mask_border);

  t->setKltOpencv(klt);
}

template<class ConfigType>
void convertVpKltOpencvToModelBasedSettingsConfig(const vpKltOpencv& klt,
           const vpMbTracker* tracker,
           ConfigType& config)
{
  const vpMbKltTracker* t = dynamic_cast<const vpMbKltTracker*>(tracker);

  config.max_features = klt.getMaxFeatures();
  config.window_size = klt.getWindowSize();
  config.quality = klt.getQuality();
  config.min_distance = klt.getMinDistance();
  config.harris = klt.getHarrisFreeParameter();
  config.size_block = klt.getBlockSize();
  config.pyramid_lvl = klt.getPyramidLevels();
  config.mask_border = t->getMaskBorder();
}

#endif //! VISP_TRACKER_CONVERSION_HH
