#ifndef HAND_EYE_CALIBRATION_H
#define HAND_EYE_CALIBRATION_H

#include <aruco.h>
#include <cvdrawingutils.h>
#include <time.h>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

#include <Eigen/Eigen>
#include <opencv2/calib3d.hpp>
#include <vector>

#include "cnpy.h"
#include "log_wrapper.h"
#include "realsense.h"

namespace calibration {

class HandEyeCalibration {
 public:
  HandEyeCalibration();
  ~HandEyeCalibration();
  ///\brief add pose expressed in the gripper frame to the robot base frame
  void AddRobotPose();

  ///\brief add pose expressed in the target frame to the camera frame
  void AddCameraPose();

  void AddCameraPose2();

  ///\brief run calibration
  void RunCalibration();

  void RunCalibration2();

  ///\brief get the homogeneous matrix that transforms a point expressed in the
  /// camera frame to the gripper frame
  void get_calibration_result();

  bool teachMode();

  bool endTeachMode();

  void RotateTest();

 private:
  int random(int x);
  void vector2matrix(cv::Mat& rvec, cv::Mat& rmatrix);

  std::ofstream outputFile1, outputFile2, outputFile3, outputFile4;
  int cnt = 0;
  driver::Realsense rs;
  double chess_length = 0.0265;  // 2.6cm
  double grid_length = 0.04;

  ur_rtde::RTDEReceiveInterface rtde_receive_;
  ur_rtde::RTDEControlInterface rtde_control_;

  std::vector<double> current_pose_;
  std::vector<cv::Mat> robot_rotation_;
  std::vector<cv::Mat> robot_translation_;
  std::vector<cv::Mat> camera_rotation_;
  std::vector<cv::Mat> camera_translation_;
  cv::Mat calibration_rotation_;
  cv::Mat calibration_translation_;
};
}  // namespace calibration
#endif