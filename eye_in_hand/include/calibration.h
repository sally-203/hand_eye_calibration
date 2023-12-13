#ifndef EYE_IN_HAND_CALIBRATION_H
#define EYE_IN_HAND_CALIBRATION_H

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

#ifdef obsensor
  #include "obsensor.h"
#endif 

#ifdef realsense
  #include "realsense.h"
#endif

//==================不要忘记修改marker_size!!!!!=========================
namespace calibration {

class EyeInHandCalibration {
 public:
  EyeInHandCalibration();

  ~EyeInHandCalibration();

  ///\brief add pose expressed in the gripper frame to the robot base frame
  void AddRobotPose();

  ///\brief add pose expressed in the target frame to the camera frame
  void AddCameraPose();

  ///\brief run calibration
  void RunCalibration();

  void RunCalibration2();

  ///\brief get the homogeneous matrix that transforms a point expressed in the
  /// camera frame to the gripper frame
  void get_calibration_result(cv::Mat& rotation, cv::Mat& translation);

  bool TeachMode();

  bool EndTeachMode();

 private:
  void vector2matrix(cv::Mat& rvec, cv::Mat& rmatrix);

  void AddPositions();

  void EstimateReproError(aruco::Marker& marker, cv::Mat& camera_matrix,
                          cv::Mat& dist_coeffs);

  std::vector<double> update(const std::vector<double>& current_pose,
                             Eigen::Vector3d& delta);
#ifdef realsense
  driver::Realsense rs;
#endif

#ifdef obsensor
  driver::Obsensor obs;
#endif

  std::string ip = "192.168.16.100";

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