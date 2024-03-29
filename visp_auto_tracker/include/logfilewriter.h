#ifndef __LOGFILEWRITER_H__
#define __LOGFILEWRITER_H__
#include <fstream>
#include <iostream>
#include <string>

namespace tracking
{
class LogFileWriter
{
private:
  std::ofstream &file_;

public:
  LogFileWriter(std::ofstream &file) : file_(file) {}
  ~LogFileWriter() { file_ << std::endl; }
  template <class T> void write(T data) { file_ << "\t" << data; }
};

} // namespace tracking
#endif /* LOGFILEWRITER_H_ */
