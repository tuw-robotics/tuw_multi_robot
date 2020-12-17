# INSTALL
We stopped supporting ROS-Kinetic

## Requirments
### System packages
```
sudo apt install libdxflib-dev

```
## ROS packages
```
export MRRP_DIR=$HOME/projects/catkin/tuw_multi_robot/
mkdir -p $MRRP_DIR/src
cd $MRRP_DIR/src
git clone git@github.com:tuw-robotics/tuw_multi_robot.git 
git clone git@github.com:tuw-robotics/tuw_msgs.git 
```

if you like to run also the demos/tutorials you also need some additional packages such as
- stage_ros
- map-server
- a CAD program such as librecad (if you like to draw your own graph using dxf files) 

Since there are not all programs and pkgs available for ubuntu 18.04 you have to compile some of them by your own.
### for Ubuntu 16.04 /  18.04

```
export ROS_VERSION=kinetic  # for Ubuntu 16.04
export ROS_VERSION=melodic  # for Ubuntu 18.04
export ROS_VERSION=noetic  # for Ubuntu 20.04

sudo apt install ros-$ROS_VERSION-map-server
sudo apt install ros-$ROS_VERSION-stage-ros
