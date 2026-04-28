T1
cd ~/IsaacSim
./isaac-sim.sh

T2
source /opt/ros/jazzy/setup.bash
cd ~/fireloop_assignment/so-arm/so101_ws
source install/setup.bash
ros2 launch so101_bringup bringup_moveit.launch.py use_fake_hardware:=true use_sim_time:=false

T3
source /opt/ros/jazzy/setup.bash
cd ~/fireloop_assignment/so-arm/so101_ws
source install/setup.bash
ros2 launch so101_state_machine so101_pipeline.launch.py

T4
source /opt/ros/jazzy/setup.bash
cd ~/fireloop_assignment/so-arm/so101_ws
source install/setup.bash
ros2 topic echo /red_cup_pose

T5
source /opt/ros/jazzy/setup.bash
cd ~/fireloop_assignment/so-arm/so101_ws
source install/setup.bash

ros2 topic pub --once /executor_command std_msgs/msg/String "{data: safe_home}"
ros2 topic pub --once /executor_command std_msgs/msg/String "{data: open}"
ros2 topic pub --once /executor_command std_msgs/msg/String "{data: plan_to_cup}"
ros2 topic pub --once /executor_command std_msgs/msg/String "{data: align_to_grasp}"
ros2 topic pub --once /executor_command std_msgs/msg/String "{data: close}"
ros2 topic pub --once /executor_command std_msgs/msg/String "{data: attach}"
ros2 topic pub --once /executor_command std_msgs/msg/String "{data: lift_cup}"
ros2 topic pub --once /executor_command std_msgs/msg/String "{data: above_box}"
ros2 topic pub --once /executor_command std_msgs/msg/String "{data: drop_in_box}"
ros2 topic pub --once /executor_command std_msgs/msg/String "{data: open}"
ros2 topic pub --once /executor_command std_msgs/msg/String "{data: detach}"
ros2 topic pub --once /executor_command std_msgs/msg/String "{data: safe_home}"

Build
source /opt/ros/jazzy/setup.bash
cd ~/fireloop_assignment/so-arm/so101_ws
colcon build --symlink-install --packages-select so101_state_machine
source install/setup.bash
