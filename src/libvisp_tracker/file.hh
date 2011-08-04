#ifndef VISP_TRACKER_FILE_HH
# define VISP_TRACKER_FILE_HH
# include <string>
# include <boost/filesystem/path.hpp>

boost::filesystem::path
getInitFileFromModelName(const std::string& modelName,
			 const std::string& defaultPath);

boost::filesystem::path
getConfigurationFileFromModelName(const std::string& modelName,
				  const std::string& configurationName,
				  const std::string& defaultPath);

boost::filesystem::path
getModelFileFromModelName(const std::string& modelName,
			  const std::string& defaultPath);

boost::filesystem::path
getConfigurationFileFromModelName(const std::string& modelName,
				  const std::string& defaultPath);

boost::filesystem::path
getInitialPoseFileFromModelName (const std::string& modelName,
				 const std::string& defaultPath);

#endif //! VISP_TRACKER_FILE_HH
