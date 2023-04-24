#ifndef VISP_TRACKER_FILE_HH
#define VISP_TRACKER_FILE_HH
#include <filesystem>
#include <string>

#include "visp_tracker/names.h"

std::string
getInitFileFromModelName( const std::string &modelName, const std::string &defaultPath );

std::string
getHelpImageFileFromModelName( const std::string &modelName, const std::string &defaultPath );

std::string
getConfigurationFileFromModelName( const std::string &modelName, const std::string &configurationName,
                                   const std::string &defaultPath );

std::string
getModelFileFromModelName( const std::string &modelName, const std::string &defaultPath );

std::string
getConfigurationFileFromModelName( const std::string &modelName, const std::string &defaultPath );

std::string
getInitialPoseFileFromModelName( const std::string &modelName, const std::string &defaultPath );

bool
makeModelFile( std::string, std::ofstream &stream, std::string &fullModelPath );

#endif //! VISP_TRACKER_FILE_HH
