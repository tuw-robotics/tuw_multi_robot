#include <ros/ros.h>
#include <simple_velocity_controller/local_multi_robot_controller_node.h>
#include <tf/transform_datatypes.h>
#include <string>
#include <algorithm>

#define NSEC_2_SECS(A) ((float)A / 1000000000.0)

int main(int argc, char **argv)
{
    if (argc >= 2)
    {
        ros::init(argc, argv, "controller"); /// initializes the ros node with default name
        ros::NodeHandle n;

        velocity_controller::LocalMultiRobotControllerNode ctrl(n);
        return 0;
    }
    else
    {
        ROS_INFO("Please specifie name \nrosrun simple_velocity_controller velocity_controller [name]");
    }
}

namespace velocity_controller
{
LocalMultiRobotControllerNode::LocalMultiRobotControllerNode(ros::NodeHandle &n) : n_(n),
                                                                             n_param_("~"),
                                                                             robot_names_(std::vector<std::string>({"robot0"}))
{
    n_param_.param("nr_of_robots", nr_of_robots_, 0);
    n_param_.param<std::string>("robot_prefix", robot_prefix_, "robot_");
    std::string robot_names_string = "";
    n_param_.param("robot_names_str", robot_names_string, robot_names_string);
    if((nr_of_robots_ == 0) && robot_names_string.empty()) {
        ROS_ERROR("The parameters nr_of_robots or robot_names_string need to be defined");
    }
    if((nr_of_robots_ > 0) && !robot_names_string.empty()) {
        ROS_ERROR("One one of the parameters nr_of_robots or robot_names_string need to be defined");
    }
    if (robot_names_string.size() > 0)
    {
        robot_names_string.erase(std::remove(robot_names_string.begin(), robot_names_string.end(), ' '), robot_names_string.end());
        std::istringstream stringStr(robot_names_string);
        std::string result;
        robot_names_.clear();
        while (std::getline(stringStr, result, ','))
        {
            robot_names_.push_back(result);
        }
        nr_of_robots_ = robot_names_.size();
    } else {
        robot_names_.resize(nr_of_robots_);
        for(int i = 0; i < nr_of_robots_; i++){
            robot_names_[i] = robot_prefix_ + std::to_string(i);
        }
        
    }

    controller.resize(robot_names_.size());
    active_robots.resize(robot_names_.size(),false);
    pubCmdVel_.resize(robot_names_.size());
    subCtrl_.resize(robot_names_.size());
    subOdom_.resize(robot_names_.size());
    subRoute_.resize(robot_names_.size());
    robot_pose_.resize(robot_names_.size());

    n_param_.param("robot_radius", robot_radius_, robot_radius_);

    //Robot radius can also be set as string and as array in yaml file
    float default_radius = 0.3;
    n_param_.param("robot_default_radius", default_radius, default_radius);
    robot_radius_.resize(robot_names_.size(), default_radius);

    topic_odom_ = "odom";
    n.getParam("odom_topic", topic_odom_);

    topic_cmdVel_ = "cmd_vel";
    n.getParam("cmd_vel_topic", topic_cmdVel_);

    topic_route_ = "route";
    n.getParam("route_topic", topic_route_);

    topic_robot_info_ = "/robot_info";
    n.getParam("robot_info_topic", topic_robot_info_);

    max_vel_v_ = 0.8;
    n.getParam("max_v", max_vel_v_);

    max_vel_w_ = 1.0;
    n.getParam("max_w", max_vel_w_);

    goal_r_ = 0.2;
    n.getParam("goal_radius", goal_r_);

    Kp_val_ = 5.0;
    n.getParam("Kp", Kp_val_);

    Ki_val_ = 0.0;
    n.getParam("Ki", Ki_val_);

    Kd_val_ = 1.0;
    n.getParam("Kd", Kd_val_);

    topic_ctrl_ = "/ctrl";
    n.getParam("topic_control", topic_ctrl_);

    n_param_.param<std::string>("frame_map", frame_map_, "map");
    
    n_param_.param<double>("update_rate", update_rate_, 20.0);
    
    n_param_.param<double>("update_rate_info", update_rate_info_, 1.0);
    
    ROS_INFO("Multi Robot Controller:  %s", topic_cmdVel_.c_str());

    for (auto &ctrl : controller)
    {
        ctrl.setSpeedParams(max_vel_v_, max_vel_w_);
        ctrl.setPID(Kp_val_, Ki_val_, Kd_val_);
        ctrl.setGoalRadius(goal_r_);
    }

    for (int i = 0; i < robot_names_.size(); i++)
    {
        pubCmdVel_[i] = n.advertise<geometry_msgs::Twist>(robot_names_[i] + "/" + topic_cmdVel_, 1);
        pubRobotInfo_ = n.advertise<tuw_multi_robot_msgs::RobotInfo>(topic_robot_info_, robot_names_.size() * 2);
        subOdom_[i] = n.subscribe<nav_msgs::Odometry>(robot_names_[i] + "/" + topic_odom_, 1, boost::bind(&LocalMultiRobotControllerNode::subOdomCb, this, _1, i));
        subRoute_[i] = n.subscribe<tuw_multi_robot_msgs::Route>(robot_names_[i] + "/" + topic_route_, 1, boost::bind(&LocalMultiRobotControllerNode::subRouteCb, this, _1, i));
        subCtrl_[i] = n.subscribe<std_msgs::String>(robot_names_[i] + "/" + topic_ctrl_, 1, boost::bind(&LocalMultiRobotControllerNode::subCtrlCb, this, _1, i));
        controller[i].setGoodId(tuw_multi_robot_msgs::RobotInfo::GOOD_EMPTY);
        controller[i].setOrderStatus(tuw_multi_robot_msgs::RobotInfo::ORDER_NONE);
        controller[i].setOrderId(-1);
    }
    subPickup_ = n.subscribe("/pickup", 10, &velocity_controller::LocalMultiRobotControllerNode::subPickupCb, this);
    ros::Rate r(update_rate_);

    int robot_info_trigger_ = 0;
    while (ros::ok())
    {
        r.sleep();
        ros::spinOnce();
        if(robot_info_trigger_ >  (update_rate_ / update_rate_info_)) {
            robot_info_trigger_ = 0;
            publishRobotInfo();
        } else {           
            robot_info_trigger_++;
        }
    }

}

void velocity_controller::LocalMultiRobotControllerNode::subOdomCb(const ros::MessageEvent<const nav_msgs::Odometry> &_event, int _topic)
{
    const nav_msgs::Odometry_<std::allocator<void>>::ConstPtr &odom = _event.getMessage();
    robot_pose_[_topic] = odom->pose;

    PathPoint pt;
    pt.x = odom->pose.pose.position.x;
    pt.y = odom->pose.pose.position.y;

    tf::Quaternion q(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    pt.theta = yaw;

    ros::Time time = ros::Time::now();
    ros::Duration d = time - last_update_;

    float delta_t = d.sec + NSEC_2_SECS(d.nsec);
    controller[_topic].update(pt, delta_t);
    if (controller[_topic].getPlanActive())
    {
      active_robots[_topic] = true;
    }
    if (!controller[_topic].getPlanActive() && active_robots[_topic])
    {
      nr_of_finished_++;
      active_robots[_topic] = false;
    }

    geometry_msgs::Twist msg;

    float v, w;
    controller[_topic].getSpeed(&v, &w);
    msg.linear.x = v;
    msg.angular.z = w;

    pubCmdVel_[_topic].publish(msg);

    //Update
    PathPrecondition pc = {_topic, controller[_topic].getCount()};

    for (SegmentController &c : controller)
    {
        c.updatePrecondition(pc);
    }
}

void velocity_controller::LocalMultiRobotControllerNode::subRouteCb(const ros::MessageEvent<const tuw_multi_robot_msgs::Route> &_event, int _topic)
{
    const tuw_multi_robot_msgs::Route_<std::allocator<void>>::ConstPtr &path = _event.getMessage();

    std::vector<PathPoint> localPath;

    if (path->segments.size() == 0)
        return;

    for (const tuw_multi_robot_msgs::RouteSegment &seg : path->segments)
    {
        PathPoint pt;

        pt.x = seg.end.position.x;
        pt.y = seg.end.position.y;

        double roll, pitch, yaw;
        tf::Quaternion q(seg.end.orientation.x, seg.end.orientation.y, seg.end.orientation.z, seg.end.orientation.w);
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        pt.theta = yaw;

        for (const tuw_multi_robot_msgs::RoutePrecondition &pc : seg.preconditions)
        {
            PathPrecondition p;
            p.robot = findRobotId(pc.robot_id);
            p.stepCondition = pc.current_route_segment;
            pt.precondition.push_back(p);
        }

        localPath.push_back(pt);
    }

    controller[_topic].setPath(std::make_shared<std::vector<PathPoint>>(localPath));
    ROS_INFO("Multi Robot Controller: Got Plan");
}

int velocity_controller::LocalMultiRobotControllerNode::findRobotId(std::string _name)
{
    for (uint32_t i = 0; i < robot_names_.size(); i++)
    {
        if (robot_names_[i].compare(_name) == 0)
            return i;
    }

    return -1;
}

void velocity_controller::LocalMultiRobotControllerNode::subCtrlCb(const ros::MessageEvent<const std_msgs::String> &_event, int _topic)
{
    const std_msgs::String_<std::allocator<void>>::ConstPtr &cmd = _event.getMessage();
    std::string s = cmd->data;

    ROS_INFO("Multi Robot Controller: received %s", s.c_str());

    if (s.compare("run") == 0)
    {
        controller[_topic].setState(run);
    }
    else if (s.compare("stop") == 0)
    {
        controller[_topic].setState(stop);
    }
    else if (s.compare("step") == 0)
    {
        controller[_topic].setState(step);
    }
    else
    {
        controller[_topic].setState(run);
    }
}

void velocity_controller::LocalMultiRobotControllerNode::subPickupCb(const tuw_multi_robot_msgs::Pickup::ConstPtr& pickup)
{
    for (uint32_t i = 0; i < robot_names_.size(); i++)
    {
        if ( robot_names_[i] == pickup->robot_name )
        {
            //controller[i].setGoodId(pickup->good_id);
            controller[i].setOrderId(pickup->order_id);
            break;
        }
    }
}

void velocity_controller::LocalMultiRobotControllerNode::publishRobotInfo()
{
    for (uint32_t i = 0; i < robot_names_.size(); i++)
    {
        tuw_multi_robot_msgs::RobotInfo ri;
        ri.header.stamp = ros::Time::now();
        ri.header.frame_id = frame_map_;
        ri.robot_name = robot_names_[i];
        ri.pose = robot_pose_[i];
        ri.shape = ri.SHAPE_CIRCLE;
        ri.shape_variables.push_back(robot_radius_[i]);
        ri.sync.robot_id = robot_names_[i];
        ri.sync.current_route_segment = controller[i].getCount();
        ri.mode = ri.MODE_NA;

        ri.status = controller[i].getStatus();
        ri.good_id = controller[i].getGoodId();

        pubRobotInfo_.publish(ri);
    }
}

} // namespace velocity_controller
