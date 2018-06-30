# INSTALL
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

if you like to run also the demos/tutorials you also need some addinal packages such as
- stage_ros
- map-server
- a CAD program such as librecad (if you like to draw your own graph using dxf files) 

Since there are not all programs and pkgs avaliabve for ubuntu 18.04 you have to comple some of them by your own.
### for Ubuntu 16.04
```
export ROS_VERSION=kinetic
sudo apt install ros-$ROS_VERSION-map-server

```
### for Ubuntu 18.04
#### map-server 
since we only need the map server of the navigation stack we can to a space checkout
```
mkdir $MRRP_DIR/src/navigation
cd $MRRP_DIR/src/navigation
git init
git remote add -f origin https://github.com/ros-planning/navigation.git
git config core.sparseCheckout true
echo "map_server" >> .git/info/sparse-checkout
git pull origin melodic-devel

```
