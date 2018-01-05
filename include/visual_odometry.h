// This file defines the visual odometry class
// Author: Huili Yu
#ifndef VISUAL_ODOMETRY_VISUAL_ODOMETRY_H_
#define VISUAL_ODOMETRY_VISUAL_ODOMETRY_H_

// Headers
#include <string>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <rgbd_base.h>
#include <configuration_file_parser.h>

namespace visual_odometry
{
// Visual odometry class
class VisualOdometry
{
 public:
  /*
   * Constructor
   */
  VisualOdometry() {}

  /*
   * Constructor
   * @param file_name: the name of config file.
   */
  VisualOdometry(const std::string &file_name);

  /*
   * Initialization
   */
  void Init();

  /*
   * Execute
   */
  void Execute();

  /*
   * SavePointCloud function saves the point cloud
   * to a file.
   * @param outfile The name of the output file.
   */
  void SavePointCloud(const std::string &outfile);

  /*
   * SetCameraIntrinsicParameters function sets the camera
   * intrinsic parameters
   */
  void SetCameraIntrinsicParameters();
  
  /*
   * ReadFrame reads data into the frame structure given the 
   * frame index
   * @param idx: frame index
   */
  Frame ReadFrame(int idx);

  /*
   * ComputeNormOfTransform function computes the norm of
   * given rotation and translation vectors.
   * @param r_vec Rotation vector.
   * @param t_vec Translation vector.
   * Return the norm. 
   */
  double ComputeNormOfTransform(cv::Mat r_vec, cv::Mat t_vec);
  
  virtual ~VisualOdometry();
 private:
  CameraIntrinsicParams camera_;// Camera intrinsic parameters
  ConfigFileParser *parser_;    // Parser for reading config file
  std::string detector_name_;   // Name of featuer detector
  std::string descriptor_name_; // Name of feature descriptor
  int start_index_;             // Start index of frame
  int end_index_;               // End index of frame
  bool visualization_;          // Whether to visualize point cloud
  int min_inliers_;             // Minimum number of inliers for good matching
  double max_norm_;             // Maximum norm for transformation
  float good_match_thresh_;     // Good match threshold
  float voxel_grid_res_;        // Voxel grid resolution
  Frame last_frame_;
  PointCloud::Ptr cloud_;
  RgbdBase rgbd_base_;
};
}

#endif // VISUAL_ODOMETRY_VISUAL_ODOMETRY_H_
