#include "calibration.h"

int main() {
  util::LogWapper::ConfigLogger()->Init("realsense_test", "./log");
  calibration::HandEyeCalibration calibrate;
  // calibrate.AddCameraPose2();

  calibrate.RunCalibration();

  // bool flag1 = calibrate.teachMode();
  // std::cout << "open tech mode: " << flag1 << std::endl;
  // bool flag2 = calibrate.endTeachMode();
  // std::cout << "close tech mode: " << flag2 << std::endl;

//   calibrate.RotateTest();

  



  return 0;
}