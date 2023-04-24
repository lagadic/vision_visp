#include <cerrno>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "visp_tracker/file.h"
#include "visp_tracker/names.h"
#include <visp3/core/vpIoTools.h>

std::string
getInitFileFromModelName( const std::string &modelName, const std::string &defaultPath )
{
  return std::string() + defaultPath + "/" + modelName + "/" + modelName + ".init";
}

std::string
getHelpImageFileFromModelName( const std::string &modelName, const std::string &defaultPath )
{
  return std::string() + defaultPath + "/" + modelName + "/" + modelName + ".ppm";
}

std::string
getModelFileFromModelName( const std::string &modelName, const std::string &defaultPath )
{
  return std::string() + defaultPath + "/" + modelName + "/" + modelName;
}

std::string
getConfigurationFileFromModelName( const std::string &modelName, const std::string &defaultPath )
{
  return std::string() + defaultPath + "/" + modelName + "/" + modelName + ".xml";
}

std::string
getInitialPoseFileFromModelName( const std::string &modelName, const std::string &defaultPath )
{
  return std::string() + defaultPath + "/" + modelName + "/" + modelName + ".0.pos";
}

bool
makeModelFile( std::string modelDescription, std::ofstream &modelStream, std::string &fullModelPath )
{
  RCLCPP_DEBUG_STREAM( rclcpp::get_logger( "rclcpp" ), " Trying to load the model from the parameter server." );
  std::string tmpname;
  try
  {
    tmpname = vpIoTools::makeTempDirectory( vpIoTools::getTempPath() );
  }
  catch ( ... )
  {
    RCLCPP_ERROR_STREAM( rclcpp::get_logger( "rclcpp" ), "Failed to create the temporary directory: " );
    return false;
  }
  // From the content of the model description check if the model is in vrml or in cao format
  std::string vrml_header( "#VRML #vrml" );
  std::string cao_header( "V1" );
  std::filesystem::path path( tmpname );
  if ( modelDescription.compare( 0, 5, vrml_header, 0, 5 ) == 0 )
  {
    path /= "model.wrl";
  }
  else if ( modelDescription.compare( 0, 5, vrml_header, 6, 5 ) == 0 )
  {
    path /= "model.wrl";
  }
  else if ( modelDescription.compare( 0, 2, cao_header ) == 0 )
  {
    path /= "model.cao";
  }
  else
  {
    RCLCPP_ERROR_STREAM( rclcpp::get_logger( "rclcpp" ),
                         "Failed to get model description from: " << path << " Given model description is -"
                                                                  << modelDescription << "-" );
    return false;
  }

  fullModelPath = path.native();

  modelStream.open( path );
  if ( !modelStream.good() )
  {
    RCLCPP_ERROR_STREAM( rclcpp::get_logger( "rclcpp" ), "Failed to create the temporary file: " << path );
    return false;
  }
  modelStream << modelDescription;
  modelStream.flush();
  return true;
}
