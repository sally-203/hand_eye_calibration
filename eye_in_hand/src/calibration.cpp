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
    : rtde_receive_("192.168.16.100"),
      rtde_control_("192.168.16.100"),
      outputFile1("/home/xlh/work/dataset/handeye_txt/handeye_cIbI.txt"),
      outputFile2("/home/xlh/work/dataset/handeye_txt/handeye_cI.txt"),
      outputFile3("/home/xlh/work/dataset/handeye_txt/handeye_bI.txt"),
      outputFile4("/home/xlh/work/dataset/handeye_txt/handeye.txt") {
  // open camera
  srand((int)time(0));
  rs.StartCamera();
  // 别忘了关相机
}

HandEyeCalibration::~HandEyeCalibration() {
  rs.CloseCamera();
  outputFile1.close();
  outputFile2.close();
  outputFile3.close();
  outputFile4.close();
}

bool HandEyeCalibration::invert(cv::Mat& m1, cv::Mat& m2) {
  Eigen::Matrix4d temp;
  // Eigen::Map<Eigen::Matrix4f> temp(m1.ptr<float>(), 4, 4);
  cv::cv2eigen(m1, temp);
  // std::cout << "--------------before inverse------------\n" << temp <<
  // std::endl;
  Eigen::Matrix4d temp_ = temp.inverse();
  // std::cout << "--------------after inverse------------\n" << temp_ <<
  // std::endl;
  cv::eigen2cv(temp_, m2);
  // std::cout << "--------------after inverse:temp2------------\n" << m2 <<
  // std::endl; temp2.copyTo(m2);
  return true;
}

void HandEyeCalibration::inverseMatrix(cv::Mat& rvec, cv::Mat& tvec,
                                       cv::Mat& rmatrix, cv::Mat& tmatrix) {
  cv::Mat transfer_matrix = cv::Mat::eye(3, 3, CV_64FC1);
  cv::Mat transfer_matrix_inv = cv::Mat::eye(4, 4, CV_64FC1);
  cv::Rodrigues(rvec, transfer_matrix);

  cv::Mat temp(3, 4, CV_64FC1);
  cv::hconcat(transfer_matrix, tvec, temp);
  cv::Mat a = (cv::Mat_<double>(1, 4) << 0, 0, 0, 1);
  cv::Mat temp2(4, 4, CV_64FC1);
  cv::vconcat(temp, a, temp2);
  std::cout << "transfer_matrix: " << temp2 << std::endl;
  bool flag = invert(temp2, transfer_matrix_inv);
  std::cout << "invert flag: " << flag << std::endl;

  std::cout << "transfer_matrix_inv: " << transfer_matrix_inv << std::endl;
  rmatrix = transfer_matrix_inv(cv::Rect(0, 0, 3, 3));
  tmatrix = transfer_matrix_inv(cv::Rect(3, 0, 1, 3));
  return;
}

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
  //   for (int i = 0; i < 10; i++) {
  //     cv::circle(color_image, corners[i], 1, cv::Scalar(0, 255, 0));
  //     std::cout << i << " " << corners[i] << std::endl;
  //   }
  //   cv::imshow("circle", color_image);
  //   cv::waitKey(0);

  // get {3D} point set
  std::vector<cv::Point3f> object_points;
  LOG_INFO("height: {}, width: {}", size.height, size.width);
  //   std::vector<int> index{0, 7, 23, 40, 47};
  //   std::vector<cv::Point2f> new_corner;
  //   for(auto i:index){
  //       cv::Point3f point(i/8 * chess_length, i%8*chess_length, 0);
  //     //   cv::circle(color_image, corners[i], 1, cv::Scalar(0, 255, 0));
  //       new_corner.push_back(corners[i]);
  //     //   std::cout << i << ": " << point << std::endl;
  //     //   cv::imshow("color image", color_image);
  //     //   cv::waitKey(0);
  //       object_points.push_back(point);
  //   }
  for (int i = 0; i < size.height; i++) {
    for (int j = 0; j < size.width; j++) {
      cv::Point3f point(i * chess_length, j * chess_length, 0);
      //   cv::circle(color_image, corners[i*size.width+j], 1, cv::Scalar(0,
      //   255, 0));

      //   std::cout << i*size.width+j << ": " << point << std::endl;
      //   cv::imshow("color image", color_image);
      //   cv::waitKey(0);
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

  cv::Mat rmatrix_ = cv::Mat::eye(3, 3, CV_64FC1);
  cv::Mat tmatrix_ = cv::Mat::zeros(3, 1, CV_64FC1);
  inverseMatrix(rvec, tvec, rmatrix_, tmatrix_);
  std::cout << "rmatrix_: " << rmatrix_ << std::endl;
  // camera inverse
  if (outputFile1.is_open() && outputFile2.is_open()) {
    outputFile1 << std::to_string(cnt) << " "
                << "camera matrix\n";
    outputFile2 << std::to_string(cnt) << " "
                << "camera matrix\n";
    for (int i = 0; i < rmatrix_.rows; ++i) {
      for (int j = 0; j < rmatrix_.cols; ++j) {
        outputFile1 << rmatrix_.at<double>(i, j) << " ";
        outputFile2 << rmatrix_.at<double>(i, j) << " ";
      }
      outputFile1 << "\n";  // 换行
      outputFile2 << "\n";  // 换行
    }

    outputFile1 << "0 camera translation\n";
    outputFile2 << "0 camera translation\n";
    for (int i = 0; i < 3; i++) {
      outputFile1 << tmatrix_.at<double>(i, 0) << " ";
      outputFile2 << tmatrix_.at<double>(i, 0) << " ";
    }
    outputFile1 << "\n";
    outputFile2 << "\n";
  }

  // camera without inverse
  if (outputFile3.is_open() && outputFile4.is_open()) {
    outputFile3 << std::to_string(cnt) << " "
                << "camera matrix\n";
    outputFile4 << std::to_string(cnt) << " "
                << "camera matrix\n";
    for (int i = 0; i < rmatrix.rows; ++i) {
      for (int j = 0; j < rmatrix.cols; ++j) {
        outputFile3 << rmatrix.at<double>(i, j) << " ";
        outputFile4 << rmatrix.at<double>(i, j) << " ";
      }
      outputFile3 << "\n";  // 换行
      outputFile4 << "\n";  // 换行
    }

    outputFile3 << "0 camera translation\n";
    outputFile4 << "0 camera translation\n";
    for (int i = 0; i < 3; i++) {
      outputFile3 << tvec.at<double>(i, 0) << " ";
      outputFile4 << tvec.at<double>(i, 0) << " ";
    }
    outputFile3 << "\n";
    outputFile4 << "\n";
  }

  std::cout << "camera rotate matrix: " << rmatrix_ << std::endl;
  std::cout << "camera translate matrix: " << tmatrix_ << std::endl;

  //   cv::invert(rvec, rvec);
  camera_rotation_.push_back(rmatrix_);
  camera_translation_.push_back(tmatrix_);

  return;
}

void HandEyeCalibration::AddRobotPose() {
  // x,y,z,rx,ry,rz
  std::vector<double> pose = rtde_receive_.getActualTCPPose();
  cv::Mat r = (cv::Mat_<double>(3, 1) << pose[3], pose[4], pose[5]);
  cv::Mat t = (cv::Mat_<double>(3, 1) << pose[0], pose[1], pose[2]);

  // get robot rotation and translation vector
  cv::Mat rmatrix = cv::Mat::eye(3, 3, CV_64FC1);
  vector2matrix(r, rmatrix);

  // get camera rotation and translation vector
  cv::Mat rmatrix_ = cv::Mat::eye(3, 3, CV_64FC1);
  cv::Mat tmatrix_ = cv::Mat::zeros(3, 1, CV_64FC1);
  inverseMatrix(r, t, rmatrix_, tmatrix_);

  std::cout << "robot rotate matrix: " << rmatrix << std::endl;
  std::cout << "robot translate matrix: " << t << std::endl;

  // robot inverse
  if (outputFile1.is_open() && outputFile3.is_open()) {
    outputFile1 << std::to_string(cnt) << " "
                << "robot matrix\n";
    outputFile3 << std::to_string(cnt) << " "
                << "robot matrix\n";
    for (int i = 0; i < rmatrix_.rows; ++i) {
      for (int j = 0; j < rmatrix_.cols; ++j) {
        outputFile1 << rmatrix_.at<double>(i, j) << " ";
        outputFile3 << rmatrix_.at<double>(i, j) << " ";
      }
      outputFile1 << "\n";  // 换行
      outputFile3 << "\n";  // 换行
    }

    outputFile1 << "0 robot translation\n";
    outputFile3 << "0 robot translation\n";
    for (int i = 0; i < 3; i++) {
      outputFile1 << tmatrix_.at<double>(i, 0) << " ";
      outputFile3 << tmatrix_.at<double>(i, 0) << " ";
    }
    outputFile1 << "\n";
    outputFile3 << "\n";
  }

  // robot without inverse
  if (outputFile2.is_open() && outputFile4.is_open()) {
    outputFile2 << std::to_string(cnt) << " "
                << "robot matrix\n";
    outputFile4 << std::to_string(cnt) << " "
                << "robot matrix\n";
    for (int i = 0; i < rmatrix.rows; ++i) {
      for (int j = 0; j < rmatrix.cols; ++j) {
        outputFile2 << rmatrix.at<double>(i, j) << " ";
        outputFile4 << rmatrix.at<double>(i, j) << " ";
      }
      outputFile2 << "\n";  // 换行
      outputFile4 << "\n";  // 换行
    }

    outputFile2 << "0 robot translation\n";
    outputFile4 << "0 robot translation\n";
    for (int i = 0; i < 3; i++) {
      outputFile2 << t.at<double>(i, 0) << " ";
      outputFile4 << t.at<double>(i, 0) << " ";
    }
    outputFile2 << "\n";
    outputFile4 << "\n";
  }

  robot_translation_.push_back(t);
  robot_rotation_.push_back(rmatrix);
  cv::waitKey(0);
  return;
}

void HandEyeCalibration::AddCameraPose2() {
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
  cv::Mat cameraMatrix2 =
      (cv::Mat_<double>(3, 3) << color_intrinsics[0], color_intrinsics[1],
       color_intrinsics[2], color_intrinsics[3]);
  cv::Mat distCoeffs =
      (cv::Mat_<double>(4, 1) << distortion_coeffs[0], distortion_coeffs[1],
       distortion_coeffs[2], distortion_coeffs[3]);

  // recognize marker
  aruco::MarkerDetector mDetector;

  // aruco::MarkerDetector::Params params = mDetector.getParameters();
  // std::string thresh_method = "THRESH_ADAPTIVE";

  float min_marker_size = 0.02;  // percentage of image area
  mDetector.setDetectionMode(aruco::DM_FAST, min_marker_size);

  std::vector<aruco::Marker> markers;
  aruco::CameraParameters camParam(cameraMatrix, distCoeffs,
                                   cv::Size(640, 480));
  double marker_size = 0.191;
  mDetector.detect(color_image, markers, camParam, marker_size, false);
  if (markers.size() == 0) {
    std::cout << "no marker" << std::endl;
    return;
  } else {
    std::cout << "markers size: " << markers.size() << std::endl;
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
  cv::waitKey(0);

  std::cout << "Rvec: " << markers[0].Rvec << std::endl;
  std::cout << "Tvec: " << markers[0].Tvec << std::endl;

  cv::Mat rmatrix_ = cv::Mat::eye(3, 3, CV_64FC1);
  cv::Mat tmatrix_ = cv::Mat::zeros(3, 1, CV_64FC1);

  tmatrix_ = markers[0].Tvec;
  vector2matrix(markers[0].Rvec, rmatrix_);
  camera_rotation_.push_back(rmatrix_);
  camera_translation_.push_back(tmatrix_);
}

void HandEyeCalibration::RunCalibration() {
  // std::vector<double> target{0.11746,  -0.39848, 0.508043,
  //                            -2.79887, 0.156054, 0.0525854};

  std::vector<double> target{0.0465827, -0.195025, 0.356259, -2.97087, 0.767235, -0.062783};
  rtde_control_.moveL(target, 0.05);

  std::vector<double> current_pose = rtde_receive_.getActualTCPPose();

  int count = 0;
  for (int k = 0; k < 3; k++) {
    for (int j = 0; j < 2; j++) {
      for (int i = 0; i < 3; i++) {
        LOG_INFO("================{}==============", count);
        std::vector<double> target = current_pose;
        target[0] += grid_length * i;
        target[1] += grid_length * j;
        target[2] += grid_length * k;
        target[3] += random(1);
        target[4] += random(1);
        target[5] += random(1);
        rtde_control_.moveL(target, 0.05);
        count++;
        AddCameraPose2();
        AddRobotPose();
        if(camera_rotation_.size() > robot_rotation_.size()){
          camera_rotation_.pop_back();
          robot_translation_.pop_back();
        }
        cnt++;
      }
    }
  }

  cv::calibrateHandEye(robot_rotation_, robot_translation_, camera_rotation_,
                       camera_translation_, calibration_rotation_,
                       calibration_translation_, cv::CALIB_HAND_EYE_TSAI);

  LOG_INFO("Calibration Result: [rotation] {}, {}, {}",
           calibration_rotation_.at<double>(0, 0),
           calibration_rotation_.at<double>(1, 0),
           calibration_rotation_.at<double>(2, 0));
  LOG_INFO("Calibration Result: [translation] {}, {}, {}",
           calibration_translation_.at<double>(0, 0),
           calibration_translation_.at<double>(1, 0),
           calibration_translation_.at<double>(2, 0));
  return;
}

void HandEyeCalibration::InverseRotation(cv::Mat& inputRotation,
                                         cv::Mat& outputRotation) {
  cv::Mat rotation_matrix;
  cv::Rodrigues(inputRotation, rotation_matrix);

  cv::Rodrigues(rotation_matrix.inv(), outputRotation);
}

void HandEyeCalibration::RotateTest() {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::io::loadPLYFile("/home/xlh/work/dataset/point1.ply", *cloud);
  // target2cam
  Eigen::Affine3f transform1;
  transform1.linear() << -0.0357986565299161, 0.9981021634303419,
      -0.05010516486675126, 0.6031053808855052, -0.01840088565590825,
      -0.7974492503940472, -0.7968578014536865, -0.05876630635618862,
      -0.601302058452861;
  transform1.translation() << -0.08816424497554527, 0.03374711424392349,
      0.6134180165992116;

  Eigen::Affine3f transform2;
  transform2.linear() << -0.03334906046930697, 0.9984118638443551,
      -0.04540473874669748, 0.6020090051107585, -0.0161970266699174,
      -0.7983250053033595, -0.7977925782629025, -0.05395745047623485,
      -0.6005127772218801;
  transform2.translation() << -0.04110693628129379, 0.03449253114957267,
      0.6061028504528647;

  Eigen::Affine3f transform3;
  transform3.linear() << -0.03227670352874623, 0.998646886954723,
      -0.0407751098705797, 0.5992512061132743, -0.01331461235688092,
      -0.8004503189265227, -0.7999101239396231, -0.05027043140282783,
      -0.5980105996932414;
  transform3.translation() << -0.00697805814189647, 0.03477235878434348,
      0.5970368015540541;

  Eigen::Affine3f transform4;
  transform4.linear() << -0.03611188278226485, 0.9981669648209909,
      -0.04856585489797044, 0.6010293322289714, -0.01713332602843409,
      -0.7990432972871997, -0.7984107174395374, -0.05804446122804607,
      -0.5993089076581702;
  transform4.translation() << -0.08339223953740107, 0.007872986072261889,
      0.6545363705792319;

  // robot2gripper
  Eigen::Affine3f transform5;
  transform5.linear() << 0.9946883125254345, 0.09523933142486177,
      0.03904651937174659, 0.078342351072717, -0.9465267303205941,
      0.3129690476980185, 0.06676553717077402, -0.3082476577986454,
      -0.9489603492812264;
  transform5.translation() << -0.1296722091279745, -0.03652563121986723,
      0.3821485301356656;
  transform5.inverse();

  pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");
  viewer.setBackgroundColor(255, 255, 255);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB>
      color_handle(cloud, 255, 0, 0);
  viewer.addPointCloud(cloud, color_handle, "cloud");
  viewer.addCoordinateSystem(1.0, transform1, "coordination1");
  viewer.addCoordinateSystem(1.0, transform2, "coordination2");
  viewer.addCoordinateSystem(1.0, transform3, "coordination3");
  viewer.addCoordinateSystem(1.0, transform4, "coordination4");
  //   viewer.addCoordinateSystem(5.0, transform5, "coordination5");
  viewer.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");

  viewer.addCoordinateSystem(1.0);
  while (!viewer.wasStopped()) {
    viewer.spin();
  }
  viewer.close();

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

bool HandEyeCalibration::isRotationMatrix(const cv::Mat& R) {
  cv::Mat tmp33 = R({0, 0, 3, 3});
  cv::Mat shouldBeIdentity;

  shouldBeIdentity = tmp33.t() * tmp33;

  cv::Mat I = cv::Mat::eye(3, 3, shouldBeIdentity.type());

  return cv::norm(I, shouldBeIdentity) < 1e-6;
}

void HandEyeCalibration::vector2matrix(cv::Mat& rvec, cv::Mat& rmatrix) {
  //   std::cout << "rotation vector before matrix transfer:---------" << rvec
  //   << std::endl;
  cv::Rodrigues(rvec, rmatrix);
  return;
}

}  // namespace calibration