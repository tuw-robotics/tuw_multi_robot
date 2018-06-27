#include <ros/ros.h>
#include <simple_velocity_controller/controller_node.h>
#include <tf/transform_datatypes.h>

#define NSEC_2_SECS(A)		((float)A/1000000000.0)


int main(int argc, char** argv) {
    if(argc >= 2)
    {
      ros::init ( argc, argv, argv[1] );  /// initializes the ros node with default name
      ros::NodeHandle n; 
      
      velocity_controller::ControllerNode ctrl(n);
      ros::Rate r(20);
      
      while ( ros::ok() ) 
      {
        ros::spinOnce();
      }
      return 0;
    }
    else
    {
      ROS_INFO("Please specifie name \nrosrun simple_velocity_controller velocity_controller [name]");
    }
}


namespace velocity_controller 
{
  ControllerNode::ControllerNode(ros::NodeHandle& n) : Controller(), 
  n_(n), 
  n_param_("~")
  {	
    topic_odom_ = "odom";
    n_param_.getParam("odom_topic", topic_odom_);
    
    topic_cmdVel_ = "cmd_vel";
    n_param_.getParam("cmd_vel_topic", topic_cmdVel_);
    
    topic_path_ = "path";
    n_param_.getParam("path_topic", topic_path_);
    
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
    setPID(Kp_val_,Ki_val_,Kd_val_);
    
	topic_ctrl_ = "/ctrl";
	n_param_.getParam("topic_control", topic_ctrl_);
    
    ROS_INFO("%s", topic_cmdVel_.c_str());
    
    pubCmdVel_ = n.advertise<geometry_msgs::Twist>(topic_cmdVel_, 1000);
    subOdom_ = n.subscribe(topic_odom_, 1000, &ControllerNode::subOdomCb, this);
    subPath_ = n.subscribe(topic_path_, 1000, &ControllerNode::subPathCb, this);
	subCtrl_ = n.subscribe(topic_ctrl_, 1000, &ControllerNode::subCtrlCb, this);
  }
  
  void ControllerNode::subOdomCb(const nav_msgs::Odometry_< std::allocator< void > >::ConstPtr& _odom)
  {
    PathPoint pt;
    pt.x = _odom->pose.pose.position.x;
    pt.y = _odom->pose.pose.position.y;
    
    tf::Quaternion q(_odom->pose.pose.orientation.x, _odom->pose.pose.orientation.y, _odom->pose.pose.orientation.z, _odom->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    pt.theta = yaw;
    
    ros::Time time = ros::Time::now();
    ros::Duration d = time-last_update_;
    
    float delta_t = d.sec + NSEC_2_SECS(d.nsec);
    update(pt, delta_t);
    
    geometry_msgs::Twist msg;
    
    float v, w;
    getSpeed(&v, &w);
    msg.linear.x = v;
    msg.angular.z = w;
    
    pubCmdVel_.publish(msg);
  }

  void ControllerNode::subPathCb(const nav_msgs::Path_< std::allocator< void > >::ConstPtr& _path)
  {
    if(_path->poses.size() == 0)
      return;
    
    // start at nearest point on path to odom
    // behavior controller resends full paths, therefore
    // it is important to start from the robots location
    
    float nearest_dist = std::numeric_limits< float >::max();
    float dist = 0;
    
    auto it = _path->poses.begin();
    bool changed = true; 
    
    while(it != _path->poses.end() && changed)
    {
      dist = pow(current_pose_.x - it->pose.position.x, 2) + pow(current_pose_.y - it->pose.position.y, 2);
      if(dist < nearest_dist)
      {
        nearest_dist = dist;
        changed = true;
        it++;
      }
      else
      {
        changed = false;
      }
    }
    
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
    ROS_INFO("Multi Robot Controller: Got Plan");
  }

  void ControllerNode::subCtrlCb(const std_msgs::String _cmd)
  {	
	std::string s = _cmd.data;
	
    ROS_INFO("Multi Robot Controller: received %s", s.c_str());
	if(s.compare("run") == 0)
	{		
	  setState(run);
	}
	else if(s.compare("stop") == 0)
	{		
	  setState(stop);
	}
	else if(s.compare("step") == 0)
	{		
	  setState(step);
	}
	else 
	{		
	  setState(run);
	}
  }

}
