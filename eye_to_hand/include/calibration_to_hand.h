/*!
 * \file calibration_to_hand.h
 * \brief one mode for hand eye calibration
 *
 * Add camera pose
 * Estimate camera reprojection error
 * Add robot pose
 * Add robot positions
 * Run calibration
 * some getter
 * open & close tech mode of robot
 *
 * \author xuliheng
 * \date 2023.12.15
 */

#ifndef EYE_TO_HAND_CALIBRATION_H
#define EYE_TO_HAND_CALIBRATION_H

#include <aruco.h>
#include <cvdrawingutils.h>
#include <time.h>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

#include <Eigen/Eigen>
#include <opencv2/calib3d.hpp>
#include <vector>

#include "log_wrapper.h"

#ifdef mechmind
#include "mechmind.h"
#endif

namespace calibration {

class EyeToHandCalibration {
 public:
  ///\brief init class
  ///\param init_marker_size: the initial marker size
  ///\param init_ip: the initial robot ip
  EyeToHandCalibration(double init_marker_size, std::string init_ip);

  ~EyeToHandCalibration();

  ///\brief add robot pose expressed in the gripper frame to the robot base
  ///frame
  void AddRobotPose();

  ///\brief add robot pose expressed in the robot base frame to the gripper frame
  void AddRobotPose2();

  ///\brief add camera pose expressed in the target frame to the camera frame
  void AddCameraPose();

  ///\brief run calibration
  void RunCalibration();

  ///\brief get the homogeneous matrix that transforms a point expressed in the
  /// camera frame to the gripper frame
  ///\param rotation: 3*3 rotation matrix
  ///\param translation: 3*1 translation matrix
  void get_calibration_result(cv::Mat& rotation, cv::Mat& translation);

  ///\brief open teach mode, making robot move freely
  bool TeachMode();

  ///\brief close teach mode, robot can't move freely
  bool EndTeachMode();

 private:
  ///\brief rotation vector transfer to rotation matrix
  ///\param rvec: rotation vector
  ///\param rmatrix: rotation matrix
  void vector2matrix(cv::Mat& rvec, cv::Mat& rmatrix);

  ///\brief add robot positions already generated
  void AddPositions();

  ///\brief estimate reprojection error to evaluate camera pose
  ///\param marker: the aruco marker
  ///\param camera_matrix: the camera pose
  ///\param dist_coeffs: the distoration coeffs
  void EstimateReproError(aruco::Marker& marker, cv::Mat& camera_matrix,
                          cv::Mat& dist_coeffs);

  ///\brief update the current pose using delta. This function used in
  ///AddPositions() 
  ///\param current_pose: the current robot pose [x, y, z, rx,
  ///ry, rz] 
  ///\param delta: euler delta
  std::vector<double> update(const std::vector<double>& current_pose,
                             Eigen::Vector3d& delta);
#ifdef mechmind
  driver::MechMind camera;
#endif

  double marker_size;

  int cnt = 0;
  double angle_delta = 5;
  double translation_delta = 0.10;

  ur_rtde::RTDEReceiveInterface rtde_receive_;
  ur_rtde::RTDEControlInterface rtde_control_;

  std::vector<double> current_pose_;
  std::vector<cv::Mat> robot_rotation_;
  std::vector<cv::Mat> robot_translation_;
  std::vector<cv::Mat> camera_rotation_;
  std::vector<cv::Mat> camera_translation_;
  std::vector<std::vector<double>> positions;
  cv::Mat calibration_rotation_;
  cv::Mat calibration_translation_;
};
}  // namespace calibration
#endif
