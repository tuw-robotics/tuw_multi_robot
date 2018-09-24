#include "fake_pose_estimation/fake_pose_estimation_nodelet.h"
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

PLUGINLIB_EXPORT_CLASS(FakePoseEstimationNodelet, nodelet::Nodelet)

void FakePoseEstimationNodelet::onInit()
{
  nh_ = getPrivateNodeHandle();
  nh_ = getNodeHandle();
  
  pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 1);
  
  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom", 1, &FakePoseEstimationNodelet::odomCallback, this);
}

void FakePoseEstimationNodelet::odomCallback(const nav_msgs::OdometryConstPtr& odom)
{
  geometry_msgs::PoseWithCovarianceStamped pose;
  pose.header = odom->header;
  pose.pose.pose = odom->pose.pose;
  pose.pose.covariance = odom->pose.covariance;
  
  pose_pub_.publish(pose);
}
