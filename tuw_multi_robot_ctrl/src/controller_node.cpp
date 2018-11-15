#include <ros/ros.h>
#include <simple_velocity_controller/controller_node.h>
#include <tuw_nav_msgs/ControllerState.h>
#include <tf/transform_datatypes.h>

#define NSEC_2_SECS(A) ((float)A / 1000000000.0)

int main(int argc, char** argv)
{
  std::string name("pid_controller");
  if (argc >= 2)
  {
      name =  argv[1];
  }
    ros::init(argc, argv, argv[1]);  /// initializes the ros node with default name
    ros::NodeHandle n;

    velocity_controller::ControllerNode ctrl(n);
    
    
    return 0;
}

namespace velocity_controller
{
ControllerNode::ControllerNode(ros::NodeHandle& n) : Controller(), n_(n), n_param_("~")
{
  max_vel_v_ = 0.8;
  n_param_.getParam("max_v", max_vel_v_);

  max_vel_w_ = 1.0;
  n_param_.getParam("max_w", max_vel_w_);
  setSpeedParams(max_vel_v_, max_vel_w_);

  goal_r_ = 0.2;
  n_param_.getParam("goal_radius", goal_r_);
  setGoalRadius(goal_r_);

  Kp_val_ = 5.0;
  n_param_.getParam("Kp", Kp_val_);

  Ki_val_ = 0.0;
  n_param_.getParam("Ki", Ki_val_);

  Kd_val_ = 1.0;
  n_param_.getParam("Kd", Kd_val_);
  setPID(Kp_val_, Ki_val_, Kd_val_);

  double loop_rate;
  n_param_.param<double>("loop_rate", loop_rate, 5);
  
  pubCmdVel_ = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  pubState_ = n.advertise<tuw_nav_msgs::ControllerState>("state_trajectory_ctrl", 10);
  subPose_ = n.subscribe("pose", 1000, &ControllerNode::subPoseCb, this);
  subPath_ = n.subscribe("path", 1000, &ControllerNode::subPathCb, this);
  subCtrl_ = n.subscribe("ctrl", 1000, &ControllerNode::subCtrlCb, this);
  
    ros::Rate r(loop_rate);
    
    while (ros::ok())
    {
      ros::spinOnce();
      publishState();
      r.sleep();
    }
}

void ControllerNode::subPoseCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &_pose)
{
  PathPoint pt;
  pt.x = _pose->pose.pose.position.x;
  pt.y = _pose->pose.pose.position.y;

  tf::Quaternion q(_pose->pose.pose.orientation.x, _pose->pose.pose.orientation.y, _pose->pose.pose.orientation.z,
                   _pose->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  pt.theta = yaw;

  ros::Time time = ros::Time::now();
  ros::Duration d = time - last_update_;

  float delta_t = d.sec + NSEC_2_SECS(d.nsec);
  update(pt, delta_t);


  float v, w;
  getSpeed(&v, &w);
  cmd_.linear.x = v;
  cmd_.angular.z = w;

  
  pubCmdVel_.publish(cmd_);
}

void ControllerNode::publishState() {
    ctrl_state_.header.stamp = ros::Time::now();
    ctrl_state_.progress = getProgress();
    if(isActive()) {
        ctrl_state_.state = ctrl_state_.STATE_DRIVING;
    } else {
        ctrl_state_.state = ctrl_state_.STATE_IDLE;
    }
    pubState_.publish(ctrl_state_);    
}

void ControllerNode::subPathCb(const nav_msgs::Path_<std::allocator<void>>::ConstPtr& _path)
{
    ctrl_state_.progress_in_relation_to = _path->header.seq;
    ctrl_state_.header.frame_id =  _path->header.frame_id;
    
  if (_path->poses.size() == 0)
    return;

  // start at nearest point on path to pose
  // behavior controller resends full paths, therefore
  // it is important to start from the robots location

  float nearest_dist = std::numeric_limits<float>::max();
  float dist = 0;

  auto it = _path->poses.begin();
  /*
  bool changed = true;

  while (it != _path->poses.end() && changed)
  {
    dist = pow(current_pose_.x - it->pose.position.x, 2) + pow(current_pose_.y - it->pose.position.y, 2);
    if (dist < nearest_dist)
    {
      nearest_dist = dist;
      changed = true;
      it++;
    }
    else
    {
      changed = false;
    }
  }*/

  std::vector<PathPoint> path;
  for (; it != _path->poses.end(); ++it)
  {
    PathPoint pt;

    tf::Quaternion q(it->pose.orientation.x, it->pose.orientation.y, it->pose.orientation.z, it->pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    pt.x = it->pose.position.x;
    pt.y = it->pose.position.y;
    pt.theta = yaw;

    path.push_back(pt);
  }

  setPath(std::make_shared<std::vector<PathPoint>>(path));
}

void ControllerNode::subCtrlCb(const std_msgs::String _cmd)
{
  std::string s = _cmd.data;

  ROS_INFO("Multi Robot Controller: received %s", s.c_str());
  if (s.compare("run") == 0)
  {
    setState(run);
  }
  else if (s.compare("stop") == 0)
  {
    setState(stop);
  }
  else if (s.compare("step") == 0)
  {
    setState(step);
  }
  else
  {
    setState(run);
  }
}
}
