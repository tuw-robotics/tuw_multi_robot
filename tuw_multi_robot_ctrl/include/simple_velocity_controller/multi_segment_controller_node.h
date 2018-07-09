#ifndef MULTI_SEGMENT_CONTROLLER_NODE_H
#define MULTI_SEGMENT_CONTROLLER_NODE_H

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <tuw_multi_robot_msgs/Route.h>
#include <tuw_multi_robot_msgs/RobotInfo.h>
#include <tuw_multi_robot_msgs/Pickup.h>

#include <simple_velocity_controller/segment_controller.h>
#include <memory>

namespace velocity_controller
{
class MultiSegmentControllerNode
{
  //special class-member functions.
public:
  /**
   * @brief Construct a new Multi Segment Controller Node object
   * @param n the node handle to subscribe to topics
   */
  MultiSegmentControllerNode(ros::NodeHandle &n);
  ros::NodeHandle n_;       ///< Node handler to the root node
  ros::NodeHandle n_param_; ///< Node handler to the current node
  std::unique_ptr<ros::Rate> rate_;
  /**
     * @brief publish the robot info of each robot
     */
  void publishRobotInfo();
  // ROS Publishers
private:
  std::vector<ros::Publisher> pubCmdVel_;
  ros::Publisher pubRobotInfo_;
  std::vector<ros::Subscriber> subOdom_;
  std::vector<ros::Subscriber> subRoute_;
  std::vector<ros::Subscriber> subCtrl_;
  ros::Subscriber subPickup_;
  std::string topic_cmdVel_;
  std::string topic_odom_;
  std::string topic_route_;
  std::string topic_ctrl_;
  std::string topic_robot_info_;
  float max_vel_v_;
  float max_vel_w_;
  float goal_r_;
  float Kp_val_;
  float Ki_val_;
  float Kd_val_;
  ros::Time last_update_;
  std::vector<std::string> robot_names_;
  std::vector<float> robot_radius_;
  void subOdomCb(const ros::MessageEvent<nav_msgs::Odometry const> &_event, int _topic);
  void subRouteCb(const ros::MessageEvent<tuw_multi_robot_msgs::Route const> &_event, int _topic);
  void subCtrlCb(const ros::MessageEvent<std_msgs::String const> &_event, int _topic);
  void subPickupCb(const tuw_multi_robot_msgs::Pickup::ConstPtr&);
  std::vector<SegmentController> controller;
  int findRobotId(std::string robot_name);
};

} // namespace velocity_controller

#endif // CONTROLLER_NODE_H
