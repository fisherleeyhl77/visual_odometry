// The implementation of the ConfigFileParser class
// Author: Huili Yu
#include <configuration_file_parser.h>
#include <assert.h>
#include <iostream>
#include <fstream>

namespace visual_odometry
{
// Constructor
ConfigFileParser::ConfigFileParser(const std::string &filename)
{
  std::ifstream f(filename.c_str());
  if (!f.is_open()) {
    std::cerr<<"Failed to open the specified parameter file."<<std::endl;
    exit(EXIT_FAILURE);
  }
  std::string str;
  while (std::getline(f, str)) {
    if (str[0] == '#') {
      continue;
    }
    int pos = str.find('=');
    if (pos == std::string::npos) {
      continue;
    }
    std::string key = str.substr(0, pos);
    std::string value = str.substr(pos + 1, str.length());
    data_[key] = value;
  }
  f.close();
}

// Retrieve the data based on key
std::string ConfigFileParser::RetrieveData(std::string key)
{
  assert(!data_.empty());
  auto iter = data_.find(key);
  if (iter == data_.end()) {
    std::cerr<<"Could not parameter "<<key<<std::endl;
    return std::string("Not found!");
  }
  return iter->second;
}
}  // namespace visual_odometry
