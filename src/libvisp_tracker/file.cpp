#include <cerrno>
#include <cstdlib>
#include <iostream>
#include <string>

#include <boost/format.hpp>
#include <boost/filesystem/path.hpp>

#include <ros/ros.h>

#include "file.hh"
#include "names.hh"


boost::filesystem::path
getInitFileFromModelName (const std::string& modelName,
			  const std::string& defaultPath)
{
  boost::filesystem::path res(defaultPath);
  res /= modelName;
  res /= modelName + ".init";
  return res;
}

boost::filesystem::path
getModelFileFromModelName (const std::string& modelName,
			   const std::string& defaultPath)
{
  boost::filesystem::path res(defaultPath);
  res /= modelName;
  res /= modelName + ".wrl";
  return res;
}

boost::filesystem::path
getConfigurationFileFromModelName (const std::string& modelName,
				   const std::string& defaultPath)
{
  boost::filesystem::path res(defaultPath);
  res /= modelName;
  res /= modelName + ".xml";
  return res;
}

boost::filesystem::path
getInitialPoseFileFromModelName (const std::string& modelName,
				 const std::string& defaultPath)
{
  boost::filesystem::path res(defaultPath);
  res /= modelName;
  res /= modelName + ".0.pos";
  return res;
}

bool
makeModelFile(boost::filesystem::ofstream& modelStream,
	      std::string& fullModelPath)
{
  std::string modelDescription;
  if (!ros::param::has(visp_tracker::model_description_param))
    {
      ROS_ERROR_STREAM("Failed to initialize: no model is provided.");
      return false;
    }
  ROS_DEBUG_STREAM("Trying to load the model from the parameter server.");

  ros::param::get(visp_tracker::model_description_param, modelDescription);

  char* tmpname = strdup("/tmp/tmpXXXXXX");
  if (mkdtemp(tmpname) == NULL)
    {
      ROS_ERROR_STREAM
	("Failed to create the temporary directory: " << strerror(errno));
      return false;
    }
  boost::filesystem::path path(tmpname);
  path /= "model.wrl";
  free(tmpname);

  fullModelPath = path.external_file_string();

  modelStream.open(path);
  if (!modelStream.good())
    {
      ROS_ERROR_STREAM
	("Failed to create the temporary file: " << path);
      return false;
    }
  modelStream << modelDescription;
  modelStream.flush();
  return true;
}
