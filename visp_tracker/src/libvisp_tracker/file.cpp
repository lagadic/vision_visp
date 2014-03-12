#include <cerrno>
#include <cstdlib>
#include <iostream>
#include <string>

#include <boost/filesystem/fstream.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/format.hpp>

#include <ros/ros.h>

#include "file.hh"
#include "names.hh"


std::string
getInitFileFromModelName (const std::string& modelName,
			  const std::string& defaultPath)
{
  boost::format fmt("%1%/%2%/%2%.init");
  fmt % defaultPath % modelName;
  return fmt.str ();
}

std::string
getModelFileFromModelName (const std::string& modelName,
			   const std::string& defaultPath)
{
  boost::format fmt("%1%/%2%/%2%.wrl");
  fmt % defaultPath % modelName;
  return fmt.str ();
}

std::string
getConfigurationFileFromModelName (const std::string& modelName,
				   const std::string& defaultPath)
{
  boost::format fmt("%1%/%2%/%2%.xml");
  fmt % defaultPath % modelName;
  return fmt.str ();
}

std::string
getInitialPoseFileFromModelName (const std::string& modelName,
				 const std::string& defaultPath)
{
  boost::format fmt("%1%/%2%/%2%.0.pos");
  fmt % defaultPath % modelName;
  return fmt.str ();
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

  fullModelPath = path.native();

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
