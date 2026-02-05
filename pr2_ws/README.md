Mobile Robot (Pr2)
===
## Dependend
```
pip3 install wxpython
pip3 install transforms3d
sudo apt-get install build-essential libgtk-3-dev
sudo apt install libyaml-cpp-dev
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers
sudo apt install -y ros-humble-gazebo-ros-pkgs
sudo apt install -y ros-humble-gazebo-ros2-control

## find a install path
## install nlohmann
git clone https://github.com/nlohmann/json.git
mkdir -p json/build && cd json/build
cmake ..
make install

```
## Workspace Compilation
``` sh
colcon build --packages-select pr2_wholebody_interface
source install/setup.bash
colcon build --packages-skip pr2_wholebody_interface
source install/setup.bash
```
### Start Controller Tools & TF
``` sh
# Start RQT Controller Manager (Choose one method)
ros2 run rqt_controller_manager rqt_controller_manager
# OR
rqt --force-discover --standalone rqt_controller_manager


# Open other terminal and Start the TF publisher node
ros2 run pr2_mujoco_publisher pr2_mujoco_tf
```
### Start the Physics Simulator
``` sh
# Open other terminal and Navigate to the simulator directory and execute:
# my simulator directory and execute，your environment may diferent:
./unitree_mujoco
```
#### Instruction Guide
![功能演示](assets/pr2_show_demo1.gif)
#### Command Execution
![功能演示](assets/pr2_show_demo2.gif)
#### Related Settings
![功能演示](assets/pr2_show_demo3.gif)



## Launch ROS 2 Control
``` sh
# Open other terminal and Launch Pr2 control
ros2 launch pr2_bringup pr2_control.launch.py
```
#### Results Display
![功能演示](assets/pr2_show_demo4.gif)

## Verify Controller Status
``` sh
# Open other terminal
ros2 control list_controllers -c /pr2/controller_manager
```

