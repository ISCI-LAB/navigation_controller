# Navigation_controller
A simple implement of robot navigation task

## Install

```bash
source /opt/ros/<ros-distro>/setup.bash
mkdir -p ~/nav_ros1_ws/src
cd ~/nav_ros1_ws/src
git clone https://github.com/ISCI-LAB/navigation_controller.git
cd ~/nav_ros1_ws
rosdep install --from-paths src --ignore-src -r -y
catkin build
```

## Quickstart

1. Launch velocity control node and action server
    ```bash
    source ~/nav_ros1_ws/devel/setup.bash
    roslaunch navigation_controller navigation.launch map_frame_id:=odom
    ```
2. (Optional) Send goal by rostopic command
    ```bash
    rostopic pub /navigation_controller/send_goal/goal move_base_msgs/MoveBaseActionGoal ...
    ```

There is an example of action client with rospy in "navigation_controller/script/".

User can create your own version as the same format.
