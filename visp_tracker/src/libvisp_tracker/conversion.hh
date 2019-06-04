#ifndef VISP_TRACKER_CONVERSION_HH
# define VISP_TRACKER_CONVERSION_HH
# include <boost/optional.hpp>

# include <ros/ros.h>

# include <geometry_msgs/Transform.h>
# include <sensor_msgs/Image.h>
# include <sensor_msgs/CameraInfo.h>
# include <tf/transform_datatypes.h>

# include <visp_tracker/Init.h>

# include <visp3/core/vpConfig.h>
# include <visp3/mbt/vpMbGenericTracker.h>

# include <visp3/core/vpHomogeneousMatrix.h>
# include <visp3/core/vpCameraParameters.h>
# include <visp3/me/vpMe.h>
# include <visp3/klt/vpKltOpencv.h>

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

std::string convertVpMbTrackerToRosMessage(const vpMbGenericTracker &tracker);

std::string convertVpMeToRosMessage(const vpMbGenericTracker &tracker, const vpMe& moving_edge);

std::string convertVpKltOpencvToRosMessage(const vpMbGenericTracker &tracker, const vpKltOpencv& klt);

void vpHomogeneousMatrixToTransform(geometry_msgs::Transform& dst,
                                    const vpHomogeneousMatrix& src);

void transformToVpHomogeneousMatrix(vpHomogeneousMatrix& dst,
                                    const geometry_msgs::Transform& src);

void transformToVpHomogeneousMatrix(vpHomogeneousMatrix& dst,
                                    const tf::Transform& src);

void transformToVpHomogeneousMatrix(vpHomogeneousMatrix& dst,
                                    const geometry_msgs::Pose& src);

void convertVpMbTrackerToInitRequest(const vpMbGenericTracker &tracker,
                                     visp_tracker::Init& srv);

void convertInitRequestToVpMbTracker(const visp_tracker::Init::Request& req,
                                     vpMbGenericTracker &tracker);

void convertVpMeToInitRequest(const vpMe& moving_edge,
                              const vpMbGenericTracker &tracker,
                              visp_tracker::Init& srv);

void convertInitRequestToVpMe(const visp_tracker::Init::Request& req,
                              vpMbGenericTracker &tracker,
                              vpMe& moving_edge);

void convertVpKltOpencvToInitRequest(const vpKltOpencv& klt,
                                     const vpMbGenericTracker &tracker,
                                     visp_tracker::Init& srv);

void convertInitRequestToVpKltOpencv(const visp_tracker::Init::Request& req,
                                     vpMbGenericTracker &tracker,
                                     vpKltOpencv& klt);

void initializeVpCameraFromCameraInfo(vpCameraParameters& cam,
                                      sensor_msgs::CameraInfoConstPtr info);

// Dynamic reconfigure template functions
template<class ConfigType>
void convertModelBasedSettingsConfigToVpMbTracker(const ConfigType& config,
                                                  vpMbGenericTracker &tracker)
{
  tracker.setAngleAppear(vpMath::rad(config.angle_appear));
  tracker.setAngleDisappear(vpMath::rad(config.angle_disappear));
}

template<class ConfigType>
void convertVpMbTrackerToModelBasedSettingsConfig(const vpMbGenericTracker &tracker,
                                                  ConfigType& config)
{
  config.angle_appear = vpMath::deg(tracker.getAngleAppear());
  config.angle_disappear = vpMath::deg(tracker.getAngleDisappear());
}

template<class ConfigType>
void convertModelBasedSettingsConfigToVpMe(const ConfigType& config,
                                           vpMe& moving_edge,
                                           vpMbGenericTracker &tracker)
{
  tracker.setGoodMovingEdgesRatioThreshold(config.first_threshold);
  moving_edge.setThreshold( config.threshold );
  moving_edge.setMaskSize( config.mask_size );
  moving_edge.setRange( config.range );
  moving_edge.setMu1( config.mu1 );
  moving_edge.setMu2( config.mu2 );
  moving_edge.setSampleStep( config.sample_step );
  moving_edge.setStrip( config.strip );

  //FIXME: not sure if this is needed.
  moving_edge.initMask();
  //Reset the tracker and the node state.
  tracker.setMovingEdge(moving_edge);
}

template<class ConfigType>
void convertVpMeToModelBasedSettingsConfig(const vpMe& moving_edge,
                                           const vpMbGenericTracker &tracker,
                                           ConfigType& config)
{
  config.first_threshold = tracker.getGoodMovingEdgesRatioThreshold();
  config.threshold = moving_edge.getThreshold();
  config.mask_size = moving_edge.getMaskSize();
  config.range = moving_edge.getRange();
  config.mu1 = moving_edge.getMu1();
  config.mu2 = moving_edge.getMu2();
  config.sample_step = moving_edge.getSampleStep();
  config.strip = moving_edge.getStrip();
}

template<class ConfigType>
void convertModelBasedSettingsConfigToVpKltOpencv(const ConfigType& config,
                                                  vpKltOpencv& klt,
                                                  vpMbGenericTracker &tracker)
{
  klt.setMaxFeatures(config.max_features);
  klt.setWindowSize(config.window_size);
  klt.setQuality(config.quality);
  klt.setMinDistance(config.min_distance);
  klt.setHarrisFreeParameter(config.harris);
  klt.setBlockSize(config.size_block);
  klt.setPyramidLevels(config.pyramid_lvl);
  tracker.setKltMaskBorder((unsigned)config.mask_border);

  tracker.setKltOpencv(klt);
}

template<class ConfigType>
void convertVpKltOpencvToModelBasedSettingsConfig(const vpKltOpencv& klt,
                                                  const vpMbGenericTracker &tracker,
                                                  ConfigType& config)
{
  config.max_features = klt.getMaxFeatures();
  config.window_size = klt.getWindowSize();
  config.quality = klt.getQuality();
  config.min_distance = klt.getMinDistance();
  config.harris = klt.getHarrisFreeParameter();
  config.size_block = klt.getBlockSize();
  config.pyramid_lvl = klt.getPyramidLevels();
  config.mask_border = tracker.getKltMaskBorder();
}

#endif //! VISP_TRACKER_CONVERSION_HH
