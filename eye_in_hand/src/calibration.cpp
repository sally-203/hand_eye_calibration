#include "calibration.h"

#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <chrono>
#include <fstream>
#include <opencv2/core/eigen.hpp>

namespace calibration {

// HandEyeCalibration::HandEyeCalibration()
//     : rtde_receive_(ip), rtde_control_(ip) {
//   // open camera
//   srand((int)time(0));
//   rs.StartCamera();
// }

HandEyeCalibration::HandEyeCalibration() {
  // open camera
  srand((int)time(0));
  rs.StartCamera();
}

HandEyeCalibration::~HandEyeCalibration() { rs.CloseCamera(); }

// void HandEyeCalibration::AddRobotPose() {
//   // x,y,z,rx,ry,rz
//   std::vector<double> pose = rtde_receive_.getActualTCPPose();
//   cv::Mat r = (cv::Mat_<double>(3, 1) << pose[3], pose[4], pose[5]);
//   cv::Mat t = (cv::Mat_<double>(1, 3) << pose[0], pose[1], pose[2]);

//   // get robot rotation and translation vector
//   cv::Mat rmatrix = cv::Mat::eye(3, 3, CV_64F);
//   vector2matrix(r, rmatrix);

//   std::cout << "robot rotate matrix: \n" << rmatrix << std::endl;
//   std::cout << "robot translate matrix: \n" << t << std::endl;

//   robot_translation_.push_back(t);
//   robot_rotation_.push_back(rmatrix);

//   return;
// }

void HandEyeCalibration::AddCameraPose() {
  // get realsense image
  cv::Mat color_image;
  rs.get_color_image_(color_image);
  cv::imshow("image", color_image);
  cv::waitKey(0);

  // get intrinics and distortion coeffs
  std::vector<double> color_intrinsics;
  std::vector<double> distortion_coeffs;
  rs.get_color_intrinsics(color_intrinsics);
  rs.get_distortion_coeffs(distortion_coeffs);

  cv::Mat cameraMatrix =
      (cv::Mat_<double>(3, 3) << color_intrinsics[0], 0, color_intrinsics[2], 0,
       color_intrinsics[1], color_intrinsics[3], 0, 0, 1);
  cv::Mat distCoeffs = (cv::Mat_<double>(4, 1) << 0, 0, 0, 0);

  // recognize marker
  aruco::MarkerDetector mDetector;

  // percentage of image area
  float min_marker_size = 0.02;
  aruco::MarkerDetector::Params params = mDetector.getParameters();
  mDetector.setDetectionMode(aruco::DM_VIDEO_FAST, min_marker_size);
  mDetector.loadParamsFromFile("/home/xlh/work/dataset/params.txt");

  // calculate pose
  std::vector<aruco::Marker> markers;
  aruco::CameraParameters camParam(cameraMatrix, distCoeffs,
                                   cv::Size(640, 480));

  mDetector.detect(color_image, markers, camParam, marker_size, false);
  cv::imshow("debug_image", mDetector.getThresholdedImage());
  std::cout << "camera matrix: " << camParam.CameraMatrix << std::endl;
  std::cout << "Extrinsics matrix: " << camParam.ExtrinsicMatrix << std::endl;

  if (markers.size() == 0) {
    std::cout << "no marker" << std::endl;
    return;
  } else {
    std::cout << "marker: " << markers[0] << std::endl;
  }

  // cv::Mat Extrinsics = cv::Mat();
  if (camParam.isValid() && marker_size != -1) {
    for (std::size_t i = 0; i < markers.size(); ++i) {
      markers[i].draw(color_image, cv::Scalar(0, 0, 255), 2);
      // draw a 3d cube in each marker if there is 3d info
      aruco::CvDrawingUtils::draw3dAxis(color_image, markers[i], camParam);
      cv::imwrite("./", color_image);
      cv::imshow("image_draw", color_image);
      cv::waitKey(0);
    }
  }

  // estimate reprojection error
  auto size = markers[0].contourPoints.size();
  for (int i = 0; i < size; ++i) {
    std::cout << markers[0].contourPoints[i] << std::endl;
  }
  EstimateReproError(markers[0].Tvec, markers[0].Rvec, camParam.CameraMatrix);

  // add camera pose
  cv::Mat rmatrix_ = cv::Mat::eye(3, 3, CV_64FC1);
  cv::Mat tmatrix_ = cv::Mat::zeros(1, 3, CV_64FC1);

  tmatrix_ = markers[0].Tvec;
  vector2matrix(markers[0].Rvec, rmatrix_);

  std::cout << "camera rotation matrix: \n" << rmatrix_ << std::endl;
  std::cout << "camera rotation translation: \n" << tmatrix_ << std::endl;

  camera_rotation_.push_back(rmatrix_);
  camera_translation_.push_back(tmatrix_);

  return;
}

void HandEyeCalibration::EstimateReproError(cv::Mat& tvec, cv::Mat& rvec,
                                            cv::Mat& camera_matrix) {
  double repro_error = 0;

  std::vector<cv::Point3d> object_points{
      cv::Point3d{-marker_size / 2, marker_size / 2, 0.0},
      {marker_size / 2, marker_size / 2, 0.0},
      {marker_size / 2, -marker_size / 2, 0.0},
      {-marker_size / 2, -marker_size / 2, 0.0}};

  cv::Mat dist_coeffs{0, 0, 0, 0};
  cv::Mat image_points;
  cv::projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs,
                    image_points);

  std::cout << image_points << std::endl;
}

std::vector<double> HandEyeCalibration::update(
    const std::vector<double>& current_pose, Eigen::Vector3d& delta) {
  std::vector<double> res;
  res.push_back(current_pose[0]);
  res.push_back(current_pose[1]);
  res.push_back(current_pose[2]);

  // vector<double>--->Eigen::Isometry3d
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  // transform.translation() << current_pose[0], current_pose[1],
  // current_pose[2];
  Eigen::Matrix3d rotation;
  // rotation = Eigen::AngleAxisd(current_pose[3], Eigen::Vector3d::UnitX()) *
  //            Eigen::AngleAxisd(current_pose[4], Eigen::Vector3d::UnitY()) *
  //            Eigen::AngleAxisd(current_pose[5], Eigen::Vector3d::UnitZ());
  Eigen::Vector3d v(current_pose[3], current_pose[4], current_pose[5]);
  Eigen::AngleAxisd ra(v.norm(), v.normalized());
  rotation = ra.matrix();
  transform.linear() = rotation;

  // std::cout << delta << std::endl;
  // Eigen::Isometry3d--->rotate
  transform.rotate(
      Eigen::AngleAxisd(delta[0] * M_PI / 180, Eigen::Vector3d::UnitX()));
  transform.rotate(
      Eigen::AngleAxisd(delta[1] * M_PI / 180, Eigen::Vector3d::UnitY()));
  transform.rotate(
      Eigen::AngleAxisd(delta[2] * M_PI / 180, Eigen::Vector3d::UnitZ()));

  // Eigen::Isometry3d--->vector<double>
  Eigen::Matrix3d rot_mat = transform.rotation();
  // std::cout << rot_mat << std::endl;
  Eigen::AngleAxisd rot_axis;
  rot_axis.fromRotationMatrix(rot_mat);
  Eigen::Vector3d rot_vec = rot_axis.axis() * rot_axis.angle();
  std::cout << rot_vec << std::endl;
  res.push_back(rot_vec.x());
  res.push_back(rot_vec.y());
  res.push_back(rot_vec.z());

  return res;
}

// void HandEyeCalibration::AddPositions() {
//   std::vector<double> current_pose = rtde_receive_.getActualTCPPose();

//   Eigen::Matrix3d basis = Eigen::Matrix3d::Identity();
//   for (int j = 0; j < 3; ++j) {
//     Eigen::Vector3d pos_delta = angle_delta * basis.col(j);
//     Eigen::Vector3d neg_delta = -angle_delta * basis.col(j);

//     positions.push_back(update(current_pose, pos_delta));
//     positions.push_back(update(current_pose, neg_delta));
//   }

//   for (int j = 0; j < 3; ++j) {
//     Eigen::Vector3d pos_delta = (angle_delta / 2) * basis.col(j);
//     Eigen::Vector3d neg_delta = (-angle_delta / 2) * basis.col(j);

//     positions.push_back(update(current_pose, pos_delta));
//     positions.push_back(update(current_pose, neg_delta));
//   }

//   std::vector<double> fp = current_pose;
//   fp[0] += translation_delta / 2;
//   positions.push_back(fp);

//   fp = current_pose;
//   fp[0] += translation_delta;
//   positions.push_back(fp);

//   fp = current_pose;
//   fp[1] -= translation_delta / 2;
//   positions.push_back(fp);

//   fp = current_pose;
//   fp[1] -= translation_delta;
//   positions.push_back(fp);

//   fp = current_pose;
//   fp[2] += translation_delta / 3;
//   positions.push_back(fp);

//   return;
// }

// void HandEyeCalibration::RunCalibration() {
//   // std::vector<double> target{0.0465827, -0.195025, 0.506259,
//   //                            -2.97087,  0.767235,  -0.062783};
//   std::vector<double> target{0.0221659, -0.175608, 0.502187,
//                              -2.83248,  1.17237,   -0.467633};
//   rtde_control_.moveL(target, 0.01);

//   AddPositions();
//   for (int i = 0; i < positions.size(); ++i) {
//     std::cout << "position: ";
//     for (int j = 0; j < positions[i].size(); ++j) {
//       std::cout << positions[i][j] << " ";
//     }
//     std::cout << std::endl;
//   }

//   // calibration
//   for (int i = 0; i < positions.size(); ++i) {
//     LOG_INFO("================{}==============", i);
//     rtde_control_.moveL(positions[i], 0.01);
//     AddCameraPose();
//     AddRobotPose();

//     if (camera_rotation_.size() < robot_rotation_.size()) {
//       robot_rotation_.pop_back();
//       robot_translation_.pop_back();
//     }

//     cnt++;
//   }

//   cv::calibrateHandEye(robot_rotation_, robot_translation_, camera_rotation_,
//                        camera_translation_, calibration_rotation_,
//                        calibration_translation_, cv::CALIB_HAND_EYE_PARK);

//   std::cout << "calibration translation: \n"
//             << calibration_translation_ << std::endl;
//   std::cout << "calibration rotation: \n" << calibration_rotation_ <<
//   std::endl;

//   return;
// }

// void HandEyeCalibration::RunCalibration2() {
//   cnpy::NpyArray hand_world_rot =
//       cnpy::npy_load("/home/xlh/work/dataset/easyhandeye/5/hand_world_rot.npy");
//   cnpy::NpyArray hand_world_tr =
//       cnpy::npy_load("/home/xlh/work/dataset/easyhandeye/5/hand_world_tr.npy");
//   std::vector<size_t> shape = hand_world_rot.shape;
//   std::cout << "Shape: ";
//   for (size_t dim : shape) {
//     std::cout << dim << " ";
//   }
//   std::cout << std::endl;

//   double* data_rot = hand_world_rot.data<double>();
//   double* data_tr = hand_world_tr.data<double>();

//   std::cout << "Data:" << std::endl;
//   for (size_t i = 0; i < shape[0]; ++i) {
//     std::cout << "Matrix " << i << ":" << std::endl;
//     for (size_t j = 0; j < shape[1]; ++j) {
//       for (size_t k = 0; k < shape[2]; ++k) {
//         std::cout << data_rot[i * shape[1] * shape[2] + j * shape[2] + k]
//                   << " ";
//       }
//       std::cout << std::endl;
//     }
//     std::cout << std::endl;
//   }

//   int count = 0;
//   LOG_INFO("================{}==============", count);

//   std::vector<std::vector<double>> position;
//   for (size_t i = 0; i < shape[0]; ++i) {
//     // rotation matrix ----> rotation vector
//     Eigen::Matrix3d rotationMatrix =
//         Eigen::Map<Eigen::Matrix3d>(data_rot + i * 9);
//     Eigen::Matrix3d rotationMatrix2 = rotationMatrix.transpose();
//     Eigen::AngleAxisd rotationAxis(rotationMatrix2);
//     Eigen::Vector3d rotationVector = rotationAxis.axis() *
//     rotationAxis.angle();

//     // eigen: rotation matrix ---> mat: rotation matrix
//     cv::Mat cvRotationMatrix = cv::Mat(3, 3, CV_64F);
//     cv::eigen2cv(rotationMatrix2, cvRotationMatrix);
//     // robot_rotation_.push_back(cvRotationMatrix);

//     // translation matrix ---> translation vector ----> cv::mat
//     Eigen::Vector3d vectorData = Eigen::Map<Eigen::Vector3d>(data_tr + i *
//     3); cv::Mat cvVector = cv::Mat(1, 3, CV_64F); cv::eigen2cv(vectorData,
//     cvVector);
//     // robot_translation_.push_back(cvVector);

//     // transform to base
//     Eigen::Matrix4d hoMatrix;
//     hoMatrix << rotationMatrix2, vectorData, 0, 0, 0, 1;
//     Eigen::Matrix4d z;
//     z << -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
//     Eigen::Matrix4d invHoMatrix = z * hoMatrix;

//     Eigen::Matrix3d roInv = invHoMatrix.block<3, 3>(0, 0);
//     Eigen::AngleAxisd rotationAxis2(roInv);
//     Eigen::Vector3d rotationVector2 =
//         rotationAxis2.axis() * rotationAxis2.angle();

//     // add position
//     position.push_back({invHoMatrix(0, 3), invHoMatrix(1, 3), invHoMatrix(2,
//     3),
//                         rotationVector2[0], rotationVector2[1],
//                         rotationVector2[2]});
//   }

//   for (int i = 0; i < position.size(); i++) {
//     std::cout << "position " << i << std::endl;
//     for (int j = 0; j < position[0].size(); j++) {
//       std::cout << position[i][j] << " ";
//     }
//     std::cout << std::endl;
//   }

//   // rtde_control_.moveL(position[0], 0.05);
//   for (size_t i = 0; i < position.size(); ++i) {
//     LOG_INFO("================{}==============", count);
//     rtde_control_.moveL(position[i], 0.01);

//     AddCameraPose();
//     AddRobotPose();
//     if (camera_rotation_.size() < robot_rotation_.size()) {
//       robot_rotation_.pop_back();
//       robot_translation_.pop_back();
//     }
//     count++;
//     cnt++;
//   }

//   cv::calibrateHandEye(robot_rotation_, robot_translation_, camera_rotation_,
//                        camera_translation_, calibration_rotation_,
//                        calibration_translation_, cv::CALIB_HAND_EYE_PARK);

//   std::cout << "calibration translation: \n"
//             << calibration_translation_ << std::endl;
//   std::cout << "calibration rotation: \n" << calibration_rotation_ <<
//   std::endl;

//   return;
// }

// bool HandEyeCalibration::TeachMode() {
//   std::vector<double> current_pose;
//   current_pose = rtde_receive_.getActualTCPPose();
//   for (int i = 0; i < current_pose.size(); i++) {
//     std::cout << current_pose[i] << " " << std::endl;
//   }
//   bool flag = false;
//   // bool flag = rtde_control_.teachMode();
//   return flag;
// }

// bool HandEyeCalibration::EndTeachMode() {
//   std::vector<double> current_pose;
//   current_pose = rtde_receive_.getActualTCPPose();
//   for (int i = 0; i < current_pose.size(); i++) {
//     std::cout << current_pose[i] << " " << std::endl;
//   }
//   bool flag = rtde_control_.endTeachMode();
//   return flag;
// }

void HandEyeCalibration::vector2matrix(cv::Mat& rvec, cv::Mat& rmatrix) {
  cv::Rodrigues(rvec, rmatrix);
  return;
}

}  // namespace calibration