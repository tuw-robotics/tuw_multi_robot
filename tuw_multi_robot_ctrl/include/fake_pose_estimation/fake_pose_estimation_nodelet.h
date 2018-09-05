#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

class FakePoseEstimationNodelet : public nodelet::Nodelet
{
public:
  virtual void onInit();
  
private:
  ros::NodeHandle nh_;
  ros::NodeHandle n_param_;
  ros::Publisher pose_pub_;
  ros::Subscriber odom_sub_;
  
  void odomCallback(const nav_msgs::OdometryConstPtr& odom);
};
