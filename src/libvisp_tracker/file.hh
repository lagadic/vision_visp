#ifndef VISP_TRACKER_FILE_HH
# define VISP_TRACKER_FILE_HH
# include <string>
# include <boost/filesystem/fstream.hpp>
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

bool
makeModelFile(boost::filesystem::ofstream& stream, std::string& fullModelPath);

#endif //! VISP_TRACKER_FILE_HH
