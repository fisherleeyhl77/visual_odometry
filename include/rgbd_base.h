// This file defines data types and class that are used
// in the visual odometry.
// Author: Huili Yu
#ifndef VISUAL_ODOMETRY_RGBD_BASE_H_
#define VISUAL_ODOMETRY_RGBD_BASE_H_

// Headers
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace visual_odometry
{
// Types
typedef pcl::PointXYZRGBA Point7D;
typedef pcl::PointCloud<Point7D> PointCloud;

// Structs
// Camera intrinsic parameters
struct CameraIntrinsicParams
{
  double fx, fy, cx, cy, scale;
};
// Frame
struct Frame
{
  cv::Mat rgb_img;
  cv::Mat depth_img;
  cv::Mat descriptor;
  std::vector<cv::KeyPoint> keypoint;
};
// Transformation between frames
struct Transform
{
  cv::Mat r_vec;
  cv::Mat t_vec;
  int n_inliers;
};

// Class
class RgbdBase
{
 public:
  RgbdBase() {} 
  /*
   * ImageToPointCloud function converts rgb and depth images to point cloud.
   * @param rgb_img Input RGB image.
   * @param depth_img Input depth image.
   * @param camera Intrinsic parameters of camera.
   * Return point cloud of which each point contains x, y, z, r, g, b, 
   * and alpha values.
   */
  PointCloud::Ptr ImageToPointCloud(const cv::Mat &rgb_image, 
				    const cv::Mat &depth_image,
                                    const CameraIntrinsicParams &camera);

  /*
   * Point2dTo3d function coverts an image point with depth to a 3d point.
   * @param point Point (u, v, d).
   * @param Intrinsic parameters of camera.
   * Return a three dimensional point (x, y, z).
   */
  cv::Point3f Point2dTo3d(const cv::Point3f &point,
                          const CameraIntrinsicParams &camera);

  /*
   * ComputeKeypointsAndDescriptors function computes key points and 
   * descriptors.
   * @param detector_name the name of feature detector.
   * @param descriptor_name the name of feature descriptor.
   * @param frame Key frame that is used to estimate motion.
   */
  void ComputeKeypointsAndDescriptors(std::string detector_name,
                                      std::string descriptor_name,
                                      Frame &frame);

  /*
   * EstimationMotion function determines the transformation between 
   * two key frames.
   * @param frame1 The first key frame.
   * @param frame2 The second key frame.
   * @param camera The camera intrinsic parameters.
   * @param good_match_thresh The threshold for good feature matching
   * Return the transformation between two frames
   */
  Transform EstimateMotion(const Frame &frame1, 
                           const Frame &frame2,
                           const CameraIntrinsicParams &camera,
                           float good_match_thresh);

  /*
   * CvMat2Eigen function converts Mat in opencv to matrix in Eigen.
   * @param r_vec Rotation vector.
   * @param t_vec Translation vector.
   * Return tranformation matrix in Eigen.
   */
  Eigen::Isometry3d Mat2Eigen(cv::Mat& r_vec, cv::Mat& t_vec);

  /*
   * JoinPointCloud function combines two point clouds.
   * @param cloud Original point cloud that is going to be transformed.
   * @param new_frame New frame.
   * @param Trans Transformation matrix.
   * @param camera Camera intrinsic parameters.
   * @param voxel_grid_res The resolution of the grid.
   * Return combined point cloud.
   */
  PointCloud::Ptr JoinPointCloud(PointCloud::Ptr cloud, 
			         Frame &new_frame, 
			         Eigen::Isometry3d Trans, 
			         CameraIntrinsicParams &camera,
                                 float voxel_grid_res);
  virtual ~RgbdBase() {}
};
}  // namespace visual_odometry

#endif  // VISUAL_ODOMETRY_RGBD_BASE_H_
