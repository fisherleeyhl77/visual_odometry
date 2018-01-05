// This file defines a class for parsing the configuration
// file for Visual Odometry
// Author: Huili Yu
#ifndef VISUAL_ODOMETRY_CONFIGURATION_FILE_PARSER_H_
#define VISUAL_ODOMETRY_CONFIGURATION_FILE_PARSER_H_
#include <string>
#include <unordered_map>

namespace visual_odometry
{
class ConfigFileParser
{
 public:
  /*
   * Constructor
   */
  ConfigFileParser() {}

  /*
   * Constructor
   * @param filename: the name of config file
   */
  ConfigFileParser(const std::string &filename);

  /*
   * RetrieveData function retrieves configuration data 
   * from the file using the key
   * @param key: data key
   */
  std::string RetrieveData(std::string key);

  /*
   * Destructor
   */
  virtual ~ConfigFileParser() {}
 private:
  // Hash table to store key-value pair of the data
  std::unordered_map<std::string, std::string> data_;
};
}  // namespace visual_odometry

#endif // VISUAL_ODOMETRY_CONFIGURATION_FILE_PARSER_H_

