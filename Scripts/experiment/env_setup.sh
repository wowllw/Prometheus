#!/bin/bash
# Same ROS / PX4 / Gazebo environment as ~/.bashrc (lines 120-136). Source this AFTER any other
# setup.bash: catkin setup files reset ROS_PACKAGE_PATH, which drops the PX4 entries and makes
# "roslaunch prometheus_gazebo ..." fail with "Resource not found: px4".
source /opt/ros/noetic/setup.bash
source ~/prometheus_mavros/devel/setup.bash
[ -f ~/spirecv-ros/devel/setup.bash ] && source ~/spirecv-ros/devel/setup.bash
source ~/Prometheus/devel/setup.bash
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/Prometheus/devel/lib
for d in uav_models ugv_models sensor_models scene_models texture; do
  export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/Prometheus/Simulator/gazebo_simulator/gazebo_models/$d
done
source ~/prometheus_px4/Tools/setup_gazebo.bash ~/prometheus_px4 ~/prometheus_px4/build/amovlab_sitl_default > /dev/null
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/prometheus_px4:~/prometheus_px4/Tools/sitl_gazebo
