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
#include "mechmind.h"

///==================不要忘记修改marker_size!!!!!=========================
namespace calibration {

class EyeToHandCalibration {
 public:
  EyeToHandCalibration();

  ~EyeToHandCalibration();

  ///\brief add pose expressed in the gripper frame to the robot base frame
  void AddRobotPose();

  void AddRobotPose2();

  ///\brief add pose expressed in the target frame to the camera frame
  void AddCameraPose();

  ///\brief run calibration
  void RunCalibration();

  ///\brief get the homogeneous matrix that transforms a point expressed in the
  /// camera frame to the gripper frame
  void get_calibration_result(cv::Mat& rotation, cv::Mat& translation);

  bool TeachMode();

  bool EndTeachMode();

 private:
  void vector2matrix(cv::Mat& rvec, cv::Mat& rmatrix);

  void AddPositions();

  void EstimateReproError(aruco::Marker& marker, cv::Mat& camera_matrix, cv::Mat& dist_coeffs);

  std::vector<double> update(const std::vector<double>& current_pose,
                             Eigen::Vector3d& delta);
  // #ifdef mechmind
    driver::MechMind camera;
  // #endif
  
  std::string ip = "192.168.1.100";

  int cnt = 0;
  double marker_size = 0.140;
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
