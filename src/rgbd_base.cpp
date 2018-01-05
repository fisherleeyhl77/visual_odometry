// This file implements the RgbdBase class.
// Author: Huili Yu

// Headers
#include "rgbd_base.h"
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/core/eigen.hpp>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

namespace visual_odometry
{
// Converts rgb and depth images to point cloud
PointCloud::Ptr RgbdBase::ImageToPointCloud(
    const cv::Mat &rgb_image, const cv::Mat &depth_image,
    const CameraIntrinsicParams &camera)
{
  PointCloud::Ptr point_cloud(new PointCloud);
  
  // Go through each pixel of depth image and
  // add each point corresponding to the pixel
  // to the point cloud
  for (int r = 0; r < depth_image.rows; ++r) {
    for (int c = 0; c < depth_image.cols; ++c) {
      // Every pixel in depth image uses 16 bits
      ushort depth = depth_image.ptr<ushort>(r)[c];
      if (depth == 0) continue;
      
      // Comute 3D coordinates of the pixel
      Point7D p;
      p.z = double(depth) / camera.scale;
      p.x = (c - camera.cx) * p.z / camera.fx;
      p.y = (r - camera.cy) * p.z / camera.fy;
      
      // Assign RGB values of the 3D point
      p.b = rgb_image.ptr<uchar>(r)[c * 3];
      p.g = rgb_image.ptr<uchar>(r)[c * 3 + 1];
      p.r = rgb_image.ptr<uchar>(r)[c * 3 + 2];

      // Add p to point cloud
      point_cloud->points.push_back(p);
    }
  }
  point_cloud->height = 1;
  point_cloud->width = point_cloud->points.size();
  point_cloud->is_dense = false;
  return point_cloud;
}

// Convert an image point to 3D point
cv::Point3f RgbdBase::Point2dTo3d(const cv::Point3f &point,
                                  const CameraIntrinsicParams &camera)
{
  cv::Point3f p;
  p.z = (double)point.z / camera.scale;
  p.x = ( point.x - camera.cx) * p.z / camera.fx;
  p.y = ( point.y - camera.cy) * p.z / camera.fy;
  return p;
}

// Compute keypoints and descriptors for an input frame
void RgbdBase::ComputeKeypointsAndDescriptors(
    std::string detector_name, std::string descriptor_name, Frame &frame)
{
  // Declare feature detector and descriptor
  cv::Ptr<cv::FeatureDetector> detector;
  cv::Ptr<cv::DescriptorExtractor> descriptor;

  // Create feature detector and descriptor
  cv::initModule_nonfree();
  detector = cv::FeatureDetector::create(detector_name.c_str());
  descriptor = cv::DescriptorExtractor::create(descriptor_name.c_str());
  if (!detector || !descriptor) {
    std::cerr << "Wrong detector or descriptor type! " <<
        detector_name <<"," << descriptor_name << std::endl;
    return;
  }
  detector->detect(frame.rgb_img, frame.keypoint);
  descriptor->compute(frame.rgb_img, frame.keypoint, frame.descriptor);
}

// Estimate the transformation between two key frames
Transform RgbdBase::EstimateMotion(const Frame &frame1,
                                   const Frame &frame2,
                                   const CameraIntrinsicParams &camera,
                                   float good_match_thresh)
{
  // Feature matching
  std::vector<cv::DMatch> matches;
  cv::FlannBasedMatcher matcher;
  std::cout << frame2.descriptor.cols << " " << frame2.descriptor.rows << std::endl;
  std::cout << frame1.descriptor.cols << " " << frame1.descriptor.rows << std::endl;
  matcher.match(frame1.descriptor, frame2.descriptor, matches);
  std::cout << "Total matches: " << matches.size() << " matches." << std::endl;

  // Filter out bad matches with matching distance certain times greater 
  // than the minimum matching distance
  Transform result;
  std::vector<cv::DMatch> good_matches;
  float min_dist = FLT_MAX;
  // Get the minimum matching distance
  for (size_t i = 0; i < matches.size(); ++i) {
    min_dist = std::min(min_dist, matches[i].distance);
  }
  // Get good matches
  for (size_t i = 0; i < matches.size(); ++i) {
    if (matches[i].distance < good_match_thresh * min_dist) {
      good_matches.push_back(matches[i]);
    }
  }
  std::cout << "good matches=" << good_matches.size() << std::endl;
  if (good_matches.size() <= 5)
  {
    result.n_inliers = -1;
    return result;
  }
  
  // Store 3d points of the last image frame
  std::vector<cv::Point3f> pts_3d_first;
  // Store 2d image points of the current image frame
  std::vector<cv::Point2f> pts_img_second;

  for (size_t i = 0; i < good_matches.size(); ++i) {
    // Get keypoint coordinates of match in the first image
    cv::Point2f p = frame1.keypoint[good_matches[i].queryIdx].pt;
    // Get depth of keypoint
    ushort d = frame1.depth_img.ptr<ushort>(int(p.y))[int(p.x)];
    if (d == 0) continue;
    
    pts_img_second.push_back(cv::Point2f(
	frame2.keypoint[good_matches[i].trainIdx].pt));
    
    cv::Point3f pt(p.x, p.y, d);
    cv::Point3f pd = Point2dTo3d(pt, camera);
    pts_3d_first.push_back(pd);
  }
  if (pts_3d_first.size() ==0 || pts_img_second.size()==0)
  {
    result.n_inliers = -1;
    return result;
  }
  
  // Camera matrix
  double K[3][3] = {
    {camera.fx, 0, camera.cx},
    {0, camera.fy, camera.cy},
    {0, 0, 1}
  };
  cv::Mat camera_matrix(3, 3, CV_64F, K);
 
  // Vectors of rotation and translation
  cv::Mat r_vec, t_vec;
  // Inliers of transformation
  cv::Mat inliers;
  // Solve pnp using RANSAC
  cv::solvePnPRansac(pts_3d_first, pts_img_second, camera_matrix, 
		     cv::Mat(), r_vec, t_vec, false, 100, 1.0, 100, 
		     inliers);

  // Output results
  result.r_vec = r_vec;
  result.t_vec = t_vec;
  result.n_inliers = inliers.rows;
  return result;
}

// Convert Mat to Eigen vector
Eigen::Isometry3d RgbdBase::Mat2Eigen(cv::Mat& r_vec, cv::Mat& t_vec)
{
  // Convert rotation vector to rotation matrix
  cv::Mat R;
  cv::Rodrigues(r_vec, R);
  Eigen::Matrix3d r;
  cv::cv2eigen(R, r);

  // Convert translation vector and rotation matrix to transformation matrix
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
 
  Eigen::AngleAxisd angle(r);
  std::cout<<"translation"<<std::endl;
  Eigen::Translation<double,3> trans(
      t_vec.at<double>(0,0), 
      t_vec.at<double>(0,1), 
      t_vec.at<double>(0,2));
  T = angle;
  T(0,3) = t_vec.at<double>(0,0); 
  T(1,3) = t_vec.at<double>(0,1); 
  T(2,3) = t_vec.at<double>(0,2);
  return T;
}

// Combine two point clouds
PointCloud::Ptr RgbdBase::JoinPointCloud(PointCloud::Ptr cloud,
                                         Frame &new_frame,
                                         Eigen::Isometry3d Trans,
                                         CameraIntrinsicParams &camera,
                                         float voxel_grid_res)
{
  PointCloud::Ptr new_cloud = ImageToPointCloud(new_frame.rgb_img,
                                                new_frame.depth_img,
                                                camera);

  // Joint point clouds
  PointCloud::Ptr transformed_cloud(new PointCloud());
  pcl::transformPointCloud(*cloud, *transformed_cloud, Trans.matrix());
  *new_cloud += *transformed_cloud;

  // Downsample point cloud using voxel filter
  static pcl::VoxelGrid<Point7D> voxel;
  voxel.setLeafSize(voxel_grid_res, voxel_grid_res, voxel_grid_res);
  voxel.setInputCloud(new_cloud);
  PointCloud::Ptr output(new PointCloud());
  voxel.filter(*output);
  return output;
}

}  // namespace visual_odometry
