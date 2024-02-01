#include "calibration_in_hand.h"

int main()
{
    util::LogWapper::ConfigLogger()->Init("calibration", "./log");
    double init_marker_size = 0.140;

    // -------------------UR---------------------
    // std::string init_ip = "192.168.3.100";
    // calibration::EyeInHandCalibration calibrate(init_marker_size, init_ip);
    // calibrate.RunCalibration();

    // -------------------Flexiv---------------------
    std::string init_ip = "192.168.13.200";
    std::string local_ip = "192.168.13.210";
    calibration::EyeInHandCalibration calibrate(init_marker_size, init_ip, local_ip);

    bool flag1 = calibrate.TeachMode();
    std::cout << "open tech mode: " << flag1 << std::endl;
    // bool flag2 = calibrate.EndTeachMode();
    // std::cout << "close tech mode: " << flag2 << std::endl;
    return 0;
}
