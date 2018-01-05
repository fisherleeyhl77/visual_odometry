// This file tests the visual odometry code.
// Author: Huili Yu
#include <iostream>
#include "visual_odometry.h"

int main(int argc, char **argv)
{
  if (argc != 3) {
    std::cerr << "Useage: ./visual_odometry "
    "path_to_config_file path_to_result_file" << std::endl;
    return -1;
  }
  visual_odometry::VisualOdometry vo(argv[1]);
  vo.Init();
  vo.Execute();
  vo.SavePointCloud(argv[2]);
  std::cout << "End of visual odometry execution" << std::endl;
  return 0;
}
