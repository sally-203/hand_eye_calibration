#include "calibration.h"

#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <chrono>
#include <fstream>
#include <opencv2/core/eigen.hpp>
#include <thread>

namespace calibration {

HandEyeCalibration::HandEyeCalibration()
    : rtde_receive_("192.168.16.100"), rtde_control_("192.168.16.100") {
  // open camera
  srand((int)time(0));
  rs.StartCamera();
  // 别忘了关相机
}

HandEyeCalibration::~HandEyeCalibration() { rs.CloseCamera(); }

void HandEyeCalibration::AddCameraPose() {
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  // get realsense image
  cv::Mat color_image;
  rs.get_color_image_(color_image);
  cv::imshow("image", color_image);
  cv::waitKey(0);

  // get gray image
  cv::Mat image;
  cv::cvtColor(color_image, image, cv::COLOR_BGR2GRAY);

  // get {2D} point set, find chessboard corners
  std::vector<cv::Point2f> corners;
  cv::Size size(8, 6);
  cv::findChessboardCorners(image, size, corners,
                            cv::CALIB_CB_ADAPTIVE_THRESH +
                                cv::CALIB_CB_NORMALIZE_IMAGE +
                                cv::CALIB_CB_FAST_CHECK);
  LOG_INFO("size of corners: {}", corners.size());

  // draw the chessboard corners
  bool patternWasFound;
  cv::drawChessboardCorners(image, size, corners, patternWasFound);
  cv::imshow("Chessboard Corners", image);

  std::string path1 =
      "/home/xlh/work/dataset/handeye_image/" + std::to_string(cnt) + ".jpg";
  cv::imwrite(path1, image);

  // get {3D} point set
  std::vector<cv::Point3f> object_points;
  LOG_INFO("height: {}, width: {}", size.height, size.width);

  for (int i = 0; i < size.height; i++) {
    for (int j = 0; j < size.width; j++) {
      cv::Point3f point(i * chess_length, j * chess_length, 0);
      object_points.push_back(point);
    }
  }

  // solve pnp based on {3D} point set and {2D} point set
  std::vector<double> color_intrinsics;
  std::vector<double> distortion_coeffs;
  rs.get_color_intrinsics(color_intrinsics);
  rs.get_distortion_coeffs(distortion_coeffs);

  cv::Mat cameraMatrix =
      (cv::Mat_<double>(3, 3) << color_intrinsics[0], 0, color_intrinsics[2], 0,
       color_intrinsics[1], color_intrinsics[3], 0, 0, 1);
  cv::Mat distCoeffs =
      (cv::Mat_<double>(5, 1) << distortion_coeffs[0], distortion_coeffs[1],
       distortion_coeffs[2], distortion_coeffs[3], distortion_coeffs[4]);
  std::cout << "camera intrinsics: " << cameraMatrix << std::endl;

  cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
  cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);

  LOG_INFO("size of object points: {}, size of corners: {}",
           object_points.size(), corners.size());
  cv::solvePnP(object_points, corners, cameraMatrix, distCoeffs, rvec, tvec,
               cv::SOLVEPNP_ITERATIVE);

  cv::circle(color_image, corners[0], 1, cv::Scalar(0, 255, 0));
  cv::circle(color_image, corners[1], 1, cv::Scalar(255, 0, 0));
  cv::imshow("color_image_c", color_image);

  cv::Mat rmatrix = cv::Mat::eye(3, 3, CV_64FC1);
  vector2matrix(rvec, rmatrix);

  //   cv::invert(rvec, rvec);
  camera_rotation_.push_back(rmatrix);
  camera_translation_.push_back(tvec);

  return;
}

void HandEyeCalibration::AddRobotPose() {
  // x,y,z,rx,ry,rz
  std::vector<double> pose = rtde_receive_.getActualTCPPose();
  cv::Mat r = (cv::Mat_<double>(3, 1) << pose[3], pose[4], pose[5]);
  cv::Mat t = (cv::Mat_<double>(1, 3) << pose[0], pose[1], pose[2]);

  // get robot rotation and translation vector
  cv::Mat rmatrix = cv::Mat::eye(3, 3, CV_64F);
  vector2matrix(r, rmatrix);

  // get camera rotation and translation vector
  // cv::Mat rmatrix_ = cv::Mat::eye(3, 3, CV_64FC1);
  // cv::Mat tmatrix_ = cv::Mat::zeros(3, 1, CV_64FC1);
  // inverseMatrix(r, t, rmatrix_, tmatrix_);

  std::cout << "robot rotate matrix: " << rmatrix << std::endl;
  std::cout << "robot translate matrix: " << t << std::endl;

  robot_translation_.push_back(t);
  robot_rotation_.push_back(rmatrix);
  // cv::waitKey(0);
  return;
}

void HandEyeCalibration::AddCameraPose2() {
  // std::this_thread::sleep_for(std::chrono::milliseconds(5000));
  
  // get realsense image
  cv::Mat color_image;
  //  = cv::imread("/home/xlh/work/dataset/image.png");
  
  rs.get_color_image_(color_image);
  // cv::cvtColor(color_image, color_image, cv::COLOR_BGR2RGB);
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
  cv::Mat cameraMatrix2 =
      (cv::Mat_<double>(3, 3) << color_intrinsics[0], color_intrinsics[1],
       color_intrinsics[2], color_intrinsics[3]);
  // cv::Mat distCoeffs =
  //     (cv::Mat_<double>(4, 1) << distortion_coeffs[0], distortion_coeffs[1],
  //      distortion_coeffs[2], distortion_coeffs[3]);
  cv::Mat distCoeffs = (cv::Mat_<double>(4, 1) << 0, 0, 0, 0);

  // recognize marker
  aruco::MarkerDetector mDetector;
  aruco::MarkerDetector::Params params = mDetector.getParameters();
  std::string thresh_method;
    switch (params.thresMethod)
    {
      case aruco::MarkerDetector::ThresMethod::THRES_ADAPTIVE:
        thresh_method = "THRESH_ADAPTIVE";
        break;
      case aruco::MarkerDetector::ThresMethod::THRES_AUTO_FIXED:
        thresh_method = "THRESH_AUTO_FIXED";
        break;
      default:
        thresh_method = "UNKNOWN";
        break;
    }

    std::cout << "thresh_method: " << thresh_method << std::endl;

  float min_marker_size = 0.02;  // percentage of image area
  mDetector.setDetectionMode(aruco::DM_VIDEO_FAST, min_marker_size);

  std::vector<aruco::Marker> markers;
  aruco::CameraParameters camParam(cameraMatrix, distCoeffs,
                                   cv::Size(640, 480));
  double marker_size = 0.134;

  mDetector.loadParamsFromFile("/home/xlh/work/dataset/params.txt");
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
      cv::imshow("image_draw", color_image);
      cv::waitKey(0);
      // markers[i].calculateExtrinsics(marker_size, camParam.CameraMatrix,
      // camParam.Distorsion, camParam.ExtrinsicMatrix);
    }
  }

  // std::cout << "Rvec: " << markers[0].Rvec << std::endl;
  // std::cout << "Tvec: " << markers[0].Tvec << std::endl;

  cv::Mat rmatrix_ = cv::Mat::eye(3, 3, CV_64FC1);
  cv::Mat tmatrix_ = cv::Mat::zeros(1, 3, CV_64FC1);

  tmatrix_ = markers[0].Tvec;
  vector2matrix(markers[0].Rvec, rmatrix_);

  std::cout << "camera rotation matrix: \n" << rmatrix_ << std::endl;
  std::cout << "camera rotation translation: \n" << tmatrix_ << std::endl;

  camera_rotation_.push_back(rmatrix_);
  camera_translation_.push_back(tmatrix_);
}

void HandEyeCalibration::RunCalibration() {
  // std::vector<double> target{0.11746,  -0.39848, 0.508043,
  //                            -2.79887, 0.156054, 0.0525854};

  std::vector<double> target{0.0465827, -0.195025, 0.506259,
                             -2.97087,  0.767235,  -0.062783};
  rtde_control_.moveL(target, 0.05);

  std::vector<double> current_pose = rtde_receive_.getActualTCPPose();

  int count = 0;
  // for (int k = 0; k < 3; k++) {
  //   for (int j = 0; j < 2; j++) {
  //     for (int i = 0; i < 3; i++) {
  //       LOG_INFO("================{}==============", count);
  //       std::vector<double> target = current_pose;
  //       target[0] -= grid_length * i;
  //       target[1] -= grid_length * j;
  //       target[2] += grid_length * k;
  //       target[3] += random(10) * M_PI/180;
  //       target[5] += random(10) * M_PI/180;
  //       target[4] += random(10) * M_PI/180 ;
  //       rtde_control_.moveL(target, 0.01);
  //       count++;
  //       AddCameraPose2();
  //       AddRobotPose();
  //       if (camera_rotation_.size() < robot_rotation_.size()) {
  //         robot_rotation_.pop_back();
  //         robot_translation_.pop_back();
  //       }
  //       cnt++;
  //     }
  //   }
  // }

  for (int k = 0; k < 3; k++) {
    for (int j = 0; j < 6; j++) {
      LOG_INFO("================{}==============", count);
      std::vector<double> target = current_pose;
      target[0] -= grid_length * k;
      target[1] -= grid_length * k;;
      target[2] += grid_length * k;;
      if (j < 3) {
      target[3] += random(10) * M_PI/180;
      target[5] += random(10) * M_PI/180;
      target[4] += random(10) * M_PI/180 ;
      }
      target[3] -= random(10) * M_PI/180;
      target[5] -= random(10) * M_PI/180;
      target[4] -= random(10) * M_PI/180 ;
      rtde_control_.moveL(target, 0.01);
      count++;
      AddCameraPose2();
      AddRobotPose();
      if (camera_rotation_.size() < robot_rotation_.size()) {
        robot_rotation_.pop_back();
        robot_translation_.pop_back();
      }
      cnt++;  
    }
  }

  cv::calibrateHandEye(robot_rotation_, robot_translation_, camera_rotation_,
                       camera_translation_, calibration_rotation_,
                       calibration_translation_, cv::CALIB_HAND_EYE_TSAI);

  std::cout << "calibration translation: \n"
            << calibration_translation_ << std::endl;
  std::cout << "calibration rotation: \n" << calibration_rotation_ << std::endl;

  return;
}

void HandEyeCalibration::RunCalibration2() {
  cnpy::NpyArray hand_world_rot =
      cnpy::npy_load("/home/xlh/work/dataset/easyhandeye/5/hand_world_rot.npy");
  cnpy::NpyArray hand_world_tr =
      cnpy::npy_load("/home/xlh/work/dataset/easyhandeye/5/hand_world_tr.npy");
  std::vector<size_t> shape = hand_world_rot.shape;
  std::cout << "Shape: ";
  for (size_t dim : shape) {
    std::cout << dim << " ";
  }
  std::cout << std::endl;

  double* data_rot = hand_world_rot.data<double>();
  double* data_tr = hand_world_tr.data<double>();

  // std::cout << "Data:" << std::endl;
  // for (size_t i = 0; i < shape[0]; ++i) {
  //   std::cout << "t " << i << ":" << std::endl;
  //   for (size_t j = 0; j < shape[1]; ++j) {
  //     // for (size_t k = 0; k < shape[2]; ++k) {
  //     std::cout << data_tr[i * shape[1] + j] << " ";
  //     // }
  //     // std::cout << std::endl;
  //   }
  //   std::cout << std::endl;
  // }

  std::cout << "Data:" << std::endl;
  for (size_t i = 0; i < shape[0]; ++i) {
    std::cout << "Matrix " << i << ":" << std::endl;
    for (size_t j = 0; j < shape[1]; ++j) {
      for (size_t k = 0; k < shape[2]; ++k) {
        std::cout << data_rot[i * shape[1] * shape[2] + j * shape[2] + k]
                  << " ";
      }
      std::cout << std::endl;
    }
    std::cout << std::endl;
  }

  int count = 0;
  LOG_INFO("================{}==============", count);

  std::vector<std::vector<double>> position;
  for (size_t i = 0; i < shape[0]; ++i) {
    // rotation matrix ----> rotation vector
    Eigen::Matrix3d rotationMatrix =
        Eigen::Map<Eigen::Matrix3d>(data_rot + i * 9);
    Eigen::Matrix3d rotationMatrix2 = rotationMatrix.transpose();
    Eigen::AngleAxisd rotationAxis(rotationMatrix2);
    Eigen::Vector3d rotationVector = rotationAxis.axis() * rotationAxis.angle();

    // eigen: rotation matrix ---> mat: rotation matrix
    cv::Mat cvRotationMatrix = cv::Mat(3, 3, CV_64F);
    cv::eigen2cv(rotationMatrix2, cvRotationMatrix);
    // robot_rotation_.push_back(cvRotationMatrix);

    // translation matrix ---> translation vector ----> cv::mat
    Eigen::Vector3d vectorData = Eigen::Map<Eigen::Vector3d>(data_tr + i * 3);
    cv::Mat cvVector = cv::Mat(1, 3, CV_64F);
    cv::eigen2cv(vectorData, cvVector);
    // robot_translation_.push_back(cvVector);

    // transform to base
    Eigen::Matrix4d hoMatrix;
    hoMatrix << rotationMatrix2, vectorData, 0, 0, 0, 1;
    Eigen::Matrix4d z;
    z << -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    Eigen::Matrix4d invHoMatrix = z * hoMatrix;

    Eigen::Matrix3d roInv = invHoMatrix.block<3, 3>(0, 0);
    Eigen::AngleAxisd rotationAxis2(roInv);
    Eigen::Vector3d rotationVector2 =
        rotationAxis2.axis() * rotationAxis2.angle();

    // add position
    position.push_back({invHoMatrix(0, 3), invHoMatrix(1, 3), invHoMatrix(2, 3),
                        rotationVector2[0], rotationVector2[1],
                        rotationVector2[2]});
  }

  for (int i = 0; i < position.size(); i++) {
    std::cout << "position " << i << std::endl;
    for (int j = 0; j < position[0].size(); j++) {
      std::cout << position[i][j] << " ";
    }
    std::cout << std::endl;
  }

  // rtde_control_.moveL(position[0], 0.05);
  for (size_t i = 0; i < position.size(); ++i) {
    LOG_INFO("================{}==============", count);
    rtde_control_.moveL(position[i], 0.01);

    AddCameraPose2();
    AddRobotPose();
    if (camera_rotation_.size() < robot_rotation_.size()) {
      robot_rotation_.pop_back();
      robot_translation_.pop_back();
    }
    count++;
    cnt++;
  }

  cv::calibrateHandEye(robot_rotation_, robot_translation_, camera_rotation_,
                       camera_translation_, calibration_rotation_,
                       calibration_translation_, cv::CALIB_HAND_EYE_TSAI);

  std::cout << "calibration translation: \n"
            << calibration_translation_ << std::endl;
  std::cout << "calibration rotation: \n" << calibration_rotation_ << std::endl;

  return;
}

bool HandEyeCalibration::teachMode() {
  std::vector<double> current_pose;
  current_pose = rtde_receive_.getActualTCPPose();
  for (int i = 0; i < current_pose.size(); i++) {
    std::cout << current_pose[i] << " " << std::endl;
  }
  bool flag = rtde_control_.teachMode();
  return flag;
}

bool HandEyeCalibration::endTeachMode() {
  std::vector<double> current_pose;
  current_pose = rtde_receive_.getActualTCPPose();
  for (int i = 0; i < current_pose.size(); i++) {
    std::cout << current_pose[i] << " " << std::endl;
  }
  bool flag = rtde_control_.endTeachMode();
  return flag;
}

int HandEyeCalibration::random(int x) { return rand() % x; }

void HandEyeCalibration::vector2matrix(cv::Mat& rvec, cv::Mat& rmatrix) {
  cv::Rodrigues(rvec, rmatrix);
  return;
}

}  // namespace calibration