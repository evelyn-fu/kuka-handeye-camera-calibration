# kuka-handeye-camera-calibration
Drake script to publish kuka joint positions to ros and allow moving arm manually for handeye camera calibration.

To be used with ros moveit calibration: https://github.com/ros-planning/moveit_calibration/tree/master 

The code in this repository is meant to publish the end effector poses and iiwa base poses for camera calibration using ros2 moveit.

### Physical movement
To manually move the robot while publishing poses, run the iiwa driver in [torque only mode](https://github.com/nepfaff/iiwa_setup?tab=readme-ov-file#controlling-the-robot-in-torque_only-mode). Then, call `python run.py --use_hardware`. In the meshcat at http://localhost:7000/, there will be a button on the control panel labeled "Toggle hold/free". The robot starts in "Hold" mode and when the toggle is pressed, the gravity compensation is disabled so you can physically move the robot to a new location (make sure to be holding the robot so it does not fall down!). Then, you can press the toggle again to freeze the robot in its current location (a bit jerky, be careful). You can alternatively press the Space key to toggle. This script will continuously publish the poses to ros messages to be used by the moveit2 calibration plugin.

### Teleop
To move the robot with teleop while publishing poses, run the iiwa driver normally and call `python run_teleop.py --use_hardware`. In the meshcat at http://localhost:7000/, there will be sliders to adjust the roll/pitch/yaw/x/y/z positions of the end effector in space. This script will continuously publish the poses to ros messages to be used by the moveit2 calibration plugin.

### Recorded calibration trajectories
To automatically move the robot to preset calibration positions and take a sample at each of those locations, call `python run_recorded.py --use_hardware --camera={back_right, back_left, front}`, where `--camera` specifies which camera's calibration trajectory to play. Currently, these 3 trajectories are hardcoded for a specific setup but the code can be easily modified to add more trajectories. Then in the meshcat at http://localhost:7000/, there will be a "Start Calibration" button. Once this button is pressed, it will play the selected trajectory. Upon moving to each sample position, it will wait 2 seconds, send a signal to moveit to take a sample, wait 1 second, and then move to the next location and repeat. This script will continuously publish the poses to ros messages to be used by the moveit2 calibration plugin, regardless of if the calibration trajectory is actively running.