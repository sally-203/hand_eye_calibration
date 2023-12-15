# Hand Eye Calibration

## Descriptions
This is a hand-eye-calibration project, which has eye-in-hand and eye-to-hand mode. The supported robot is UR, and the supported camera includes realsense, obsensor, mechmind. 
Besides, it can develop further to support more robot and camera.

## Usage
- Select the calibration mode and choose camera in 'CMakeLists.txt', open the option
- Revise the 'init_marker_size' and 'init_ip' in 'test_xxx.cpp'
- Choose the right function in test_xxx.cpp, such as run_calibration, add_camera_pose,open_tech_mode, close_tech_mode etc.
- cd hand_eye_calibration && make -j
- running test

## Authors and acknowledgment


