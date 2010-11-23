#include <iostream>
#include <string>

#include "file.hh"


std::string getInitFileFromModelName (const std::string& modelName,
				      const std::string& defaultPath)
{
  std::string res(defaultPath);
  if (defaultPath[defaultPath.length () - 1] != '/')
    res += '/';
  res += modelName + "/" + modelName;
  return res;
}

std::string getModelFileFromModelName (const std::string& modelName,
				       const std::string& defaultPath)
{
  std::string res(defaultPath);
  if (defaultPath[defaultPath.length () - 1] != '/')
    res += '/';
  res += modelName + "/" + modelName + ".wrl";
  return res;
}
