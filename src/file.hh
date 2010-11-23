#ifndef VISP_TRACKER_FILE_HH
# define VISP_TRACKER_FILE_HH
# include <string>

std::string getInitFileFromModelName(const std::string& modelName,
				     const std::string& defaultPath);

std::string
getConfigurationFileFromModelName(const std::string& modelName,
				  const std::string& configurationName,
				  const std::string& defaultPath);

std::string getModelFileFromModelName(const std::string& modelName,
				      const std::string& defaultPath);

#endif //! VISP_TRACKER_FILE_HH
