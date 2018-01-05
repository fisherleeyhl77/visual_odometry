// This file implements the visual odometry class.
// Author: Huili Yu
#include "visual_odometry.h"
#include <stdlib.h>
#include <iostream>
#include <fstream>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace visual_odometry
{
// Constructor
VisualOdometry::VisualOdometry(const std::string &file_name)
{
  std::cout<<"Setting up parameters ..."<<std::endl;
  parser_ = new ConfigFileParser(file_name);
  start_index_ = atoi(parser_->RetrieveData("start_index").c_str());
  end_index_   = atoi(parser_->RetrieveData("end_index").c_str());
  detector_name_ = parser_->RetrieveData("detector");
  descriptor_name_ = parser_->RetrieveData("descriptor");
  SetCameraIntrinsicParameters();

  visualization_ = 
      parser_->RetrieveData("visualize_pointcloud")==std::string("yes");
  min_inliers_ = atoi(parser_->RetrieveData("min_inliers").c_str());
  max_norm_ = atof(parser_->RetrieveData("max_norm").c_str() );
  good_match_thresh_ = 
      atof(parser_->RetrieveData("good_match_threshold").c_str());
  voxel_grid_res_ = atof(parser_->RetrieveData("voxel_grid").c_str());
}

// Initialization
void VisualOdometry::Init()
{
  std::cout<<"Initializing ..."<<std::endl;
  last_frame_ = ReadFrame(start_index_);
  rgbd_base_.ComputeKeypointsAndDescriptors(detector_name_, 
                                            descriptor_name_,
                                            last_frame_);
  cloud_ = rgbd_base_.ImageToPointCloud(last_frame_.rgb_img,
			                last_frame_.depth_img,
                                        camera_);
}

// Execution
void VisualOdometry::Execute()
{
  pcl::visualization::CloudViewer viewer("viewer");
  for (int curr_idx = start_index_ + 1; curr_idx < end_index_; ++curr_idx) {
    std::cout << "Reading frame: " << curr_idx << std::endl;
    // Read current frame
    Frame curr_frame = ReadFrame(curr_idx);
    // Extract keypoints and descriptors
    rgbd_base_.ComputeKeypointsAndDescriptors(detector_name_,
                                              descriptor_name_, 
                                              curr_frame);
    // Compute motion between current and last frames
    Transform trans = rgbd_base_.EstimateMotion(last_frame_, curr_frame, 
                                                camera_, good_match_thresh_);
    
    // If no enough inliers of matching descriptors, skip this frame
    if (trans.n_inliers < min_inliers_) {
      continue;
    }
    // If motion is too large, skip the frame
    double norm = ComputeNormOfTransform(trans.r_vec, trans.t_vec);
    std::cout << "Norm of transform is " << norm << std::endl;
    if (norm >= max_norm_) {
      continue;
    }

    // Convert Mat returned by OpenCV to the matrix in Eigen
    Eigen::Isometry3d T = rgbd_base_.Mat2Eigen(trans.r_vec, trans.t_vec);
    std::cout<<"T="<<T.matrix()<<std::endl;

    // Joint point clouds of last and current frames
    cloud_ = rgbd_base_.JoinPointCloud(cloud_, curr_frame, T, 
                                       camera_, voxel_grid_res_);

    // Visualization
    if (visualization_) {
      viewer.showCloud(cloud_);
    }

    last_frame_ = curr_frame;
  }
}

// Save point cloud
void VisualOdometry::SavePointCloud(const std::string &outfile)
{
  pcl::io::savePCDFile(outfile, *cloud_);
}

// Set camera intrinsic parameters
void VisualOdometry::SetCameraIntrinsicParameters()
{
  camera_.fx    = atof(parser_->RetrieveData("camera.fx").c_str());
  camera_.fy    = atof(parser_->RetrieveData("camera.fy").c_str());
  camera_.cx    = atof(parser_->RetrieveData("camera.cx").c_str());
  camera_.cy    = atof(parser_->RetrieveData("camera.cy").c_str());
  camera_.scale = atof(parser_->RetrieveData("camera.scale").c_str());
}

// Read frame
Frame VisualOdometry::ReadFrame(int idx)
{
  Frame frame;
  std::string rgb_dir   =   parser_->RetrieveData("rgb_dir");
  std::string depth_dir =   parser_->RetrieveData("depth_dir");     
  std::string rgb_ext   =   parser_->RetrieveData("rgb_extension");
  std::string depth_ext =   parser_->RetrieveData("depth_extension");

  std::stringstream ss;
  ss << rgb_dir << idx << rgb_ext;
  std::string filename;
  ss >> filename;
  std::cout << filename << std::endl;
  frame.rgb_img = cv::imread(filename);

  ss.clear();
  filename.clear();
  ss << depth_dir << idx << depth_ext;
  ss >> filename;
  std::cout << filename << std::endl;
  frame.depth_img = cv::imread(filename, -1);
  return frame;
}

// Compute the norm of transformation
double VisualOdometry::ComputeNormOfTransform(cv::Mat r_vec, cv::Mat t_vec)
{
  return fabs(std::min(cv::norm(r_vec), 2*M_PI-cv::norm(r_vec))) + 
	 fabs(cv::norm(t_vec));
}

// Destructor
VisualOdometry::~VisualOdometry()
{
  delete parser_;
}

}  // namespace visual_odometry
