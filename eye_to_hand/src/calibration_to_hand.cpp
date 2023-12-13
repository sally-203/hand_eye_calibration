#include "calibration_to_hand.h"
#include <opencv2/highgui.hpp>

namespace calibration {

EyeToHandCalibration::EyeToHandCalibration()
    : rtde_receive_(ip), rtde_control_(ip) {
// open camera
#ifdef mechmind
  camera.ConnectToCamera();
#endif
}

EyeToHandCalibration::~EyeToHandCalibration() {
// close camera
#ifdef mechmind
  camera.DisconnectFromCamera();
#endif
}

void EyeToHandCalibration::AddCameraPose() {
  // get realsense image
  cv::Mat color_image;

#ifdef mechmind
  camera.GetColorImage(color_image);
#endif

  cv::imshow("image", color_image);
  cv::waitKey(0);
  cv::Size image_size = color_image.size();

  // get intrinics and distortion coeffs
  cv::Mat cameraMatrix;
  cv::Mat distCoeffs;

#ifdef mechmind
  mmind::api::CameraIntri camera_intri = camera.GetDepthCameraIntri();
  cameraMatrix = (cv::Mat_<double>(3, 3) << camera_intri.cameraMatrix[0], 0,
                  camera_intri.cameraMatrix[2], 0, camera_intri.cameraMatrix[1],
                  camera_intri.cameraMatrix[3], 0, 0, 1);
  distCoeffs = (cv::Mat_<double>(4, 1) << camera_intri.distortion[0],
                camera_intri.distortion[1], camera_intri.distortion[2],
                camera_intri.distortion[3]);
#endif

  // recognize marker
  aruco::MarkerDetector mDetector;

  // percentage of image area
  float min_marker_size = 0.02;
  aruco::MarkerDetector::Params params = mDetector.getParameters();
  std::string thresh_method;
  switch (params.thresMethod) {
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
  mDetector.setDetectionMode(aruco::DM_VIDEO_FAST, min_marker_size);
  mDetector.loadParamsFromFile("../../params.txt");

  // calculate pose
  std::vector<aruco::Marker> markers;
  aruco::CameraParameters camParam(cameraMatrix, distCoeffs, image_size);

  mDetector.detect(color_image, markers, camParam, marker_size, false);
  cv::imshow("debug_image", mDetector.getThresholdedImage());
  std::cout << "camera matrix: " << camParam.CameraMatrix << std::endl;
  std::cout << "distortion coefficients: " << camParam.Distorsion << std::endl;
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
      cv::imwrite("./" + std::to_string(cnt) + ".jpg", color_image);
      cv::imshow("image_draw", color_image);
      cv::waitKey(0);
    }
  }

  // estimate reprojection error
  EstimateReproError(markers[0], camParam.CameraMatrix, camParam.Distorsion);

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

void EyeToHandCalibration::AddRobotPose() {
  // x,y,z,rx,ry,rz
  std::vector<double> pose = rtde_receive_.getActualTCPPose();
  cv::Mat r = (cv::Mat_<double>(3, 1) << pose[3], pose[4], pose[5]);
  cv::Mat t = (cv::Mat_<double>(1, 3) << pose[0], pose[1], pose[2]);

  // get robot rotation and translation vector
  cv::Mat rmatrix = cv::Mat::eye(3, 3, CV_64F);
  vector2matrix(r, rmatrix);

  std::cout << "robot rotate matrix: \n" << rmatrix << std::endl;
  std::cout << "robot translate matrix: \n" << t << std::endl;

  robot_translation_.push_back(t);
  robot_rotation_.push_back(rmatrix);

  return;
}

void EyeToHandCalibration::AddRobotPose2() {
  // x,y,z,rx,ry,rz
  std::vector<double> pose = rtde_receive_.getActualTCPPose();
  cv::Mat transformation = cv::Mat::eye(4, 4, CV_64F);
  transformation.at<double>(0, 3) = pose[0];
  transformation.at<double>(1, 3) = pose[1];
  transformation.at<double>(2, 3) = pose[2];

  cv::Mat rotationMatrix;
  cv::Rodrigues(cv::Vec3d(pose[3], pose[4], pose[5]), rotationMatrix);

  rotationMatrix.copyTo(transformation(cv::Rect(0, 0, 3, 3)));
  cv::Mat invTransformation = transformation.inv();

  cv::Mat invRotationMatrix = invTransformation(cv::Rect(0, 0, 3, 3)).clone();
  cv::Mat invTranslationVector =
      invTransformation(cv::Rect(3, 0, 1, 3)).clone();

  robot_translation_.push_back(invTranslationVector);
  robot_rotation_.push_back(invRotationMatrix);
  return;
}

void EyeToHandCalibration::RunCalibration() {
  // mechmind
  // std::vector<double> target{0.0719977,  -0.137369,  0.565562,
  //                            -0.0684688, -0.0607518, 2.28158};
  std::vector<double> target{0.0794426, -0.112287,  0.601161,
                             -0.225432, -0.0635636, -0.997917};
  rtde_control_.moveL(target, 0.01);

  AddPositions();
  // for (int i = 0; i < positions.size(); ++i) {
  //   std::cout << "position: ";
  //   for (int j = 0; j < positions[i].size(); ++j) {
  //     std::cout << positions[i][j] << " ";
  //   }
  //   std::cout << std::endl;
  // }

  // calibration
  for (int i = 0; i < positions.size(); ++i) {
    LOG_INFO("================{}==============", i);
    rtde_control_.moveL(positions[i], 0.01);
    AddCameraPose();
    AddRobotPose2();

    if (camera_rotation_.size() < robot_rotation_.size()) {
      robot_rotation_.pop_back();
      robot_translation_.pop_back();
    }

    cnt++;
  }

  cv::calibrateHandEye(robot_rotation_, robot_translation_, camera_rotation_,
                       camera_translation_, calibration_rotation_,
                       calibration_translation_, cv::CALIB_HAND_EYE_PARK);

  std::cout << "calibration translation: \n"
            << calibration_translation_ << std::endl;
  std::cout << "calibration rotation: \n" << calibration_rotation_ << std::endl;

  return;
}

void EyeToHandCalibration::AddPositions() {
  std::vector<double> current_pose = rtde_receive_.getActualTCPPose();

  Eigen::Matrix3d basis = Eigen::Matrix3d::Identity();
  for (int j = 0; j < 3; ++j) {
    Eigen::Vector3d pos_delta = angle_delta * basis.col(j);
    Eigen::Vector3d neg_delta = -angle_delta * basis.col(j);

    positions.push_back(update(current_pose, pos_delta));
    positions.push_back(update(current_pose, neg_delta));
  }

  for (int j = 0; j < 3; ++j) {
    Eigen::Vector3d pos_delta = (angle_delta / 2) * basis.col(j);
    Eigen::Vector3d neg_delta = (-angle_delta / 2) * basis.col(j);

    positions.push_back(update(current_pose, pos_delta));
    positions.push_back(update(current_pose, neg_delta));
  }

  std::vector<double> fp = current_pose;
  fp[0] += translation_delta / 2;
  positions.push_back(fp);

  fp = current_pose;
  fp[0] += translation_delta;
  positions.push_back(fp);

  fp = current_pose;
  fp[1] -= translation_delta / 2;
  positions.push_back(fp);

  fp = current_pose;
  fp[1] -= translation_delta;
  positions.push_back(fp);

  fp = current_pose;
  fp[2] += translation_delta / 3;
  positions.push_back(fp);

  return;
}

std::vector<double> EyeToHandCalibration::update(
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

void EyeToHandCalibration::EstimateReproError(aruco::Marker& marker,
                                              cv::Mat& camera_matrix,
                                              cv::Mat& dist_coeffs) {
  double repro_error = 0;

  cv::Mat image_points;
  cv::projectPoints(marker.get3DPoints(), marker.Rvec, marker.Tvec,
                    camera_matrix, dist_coeffs, image_points);
  auto size = marker.get3DPoints().size();
  std::cout << image_points << std::endl;

  std::cout << "size: " << size << std::endl;
  for (int i = 0; i < size; ++i) {
    double e = sqrt(pow((marker[i].x - image_points.at<float>(i, 0)), 2) +
                    pow((marker[i].y - image_points.at<float>(i, 1)), 2));
    repro_error += e;
  }
  std::cout << "reproject error : " << repro_error / size << std::endl;
}

bool EyeToHandCalibration::TeachMode() {
  std::vector<double> current_pose;
  current_pose = rtde_receive_.getActualTCPPose();
  for (int i = 0; i < current_pose.size(); i++) {
    std::cout << current_pose[i] << " " << std::endl;
  }
  bool flag = false;
  // bool flag = rtde_control_.teachMode();
  return flag;
}

bool EyeToHandCalibration::EndTeachMode() {
  std::vector<double> current_pose;
  current_pose = rtde_receive_.getActualTCPPose();
  for (int i = 0; i < current_pose.size(); i++) {
    std::cout << current_pose[i] << " " << std::endl;
  }
  bool flag = rtde_control_.endTeachMode();
  return flag;
}

void EyeToHandCalibration::vector2matrix(cv::Mat& rvec, cv::Mat& rmatrix) {
  cv::Rodrigues(rvec, rmatrix);
  return;
}

void EyeToHandCalibration::get_calibration_result(cv::Mat& rotation,
                                                  cv::Mat& translation) {
  rotation = calibration_rotation_;
  translation = calibration_translation_;
  return;
}
}  // namespace calibration