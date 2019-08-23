#include <simple_velocity_controller/local_controller_node.h>

#include <tuw_nav_msgs/ControllerState.h>
#include <tf/transform_datatypes.h>

#define NSEC_2_SECS(A) ((float)A / 1000000000.0)

int main(int argc, char **argv)
{
    std::string name("pid_controller");
    if (argc >= 2) {
        name = argv[1];
    }
    ros::init(argc, argv, argv[1]);  /// initializes the ros node with default name
    ros::NodeHandle n;

    velocity_controller::ControllerNode ctrl(n);

    ros::spin();
    return 0;
}

namespace velocity_controller {
    ControllerNode::ControllerNode(ros::NodeHandle &n) : 
    n_(n),
    n_param_("~"), 
    action_server(n, "execute_path", [&](const auto& goal) {this->onGoalReceived(goal);}, false)
    {
        twist_publisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
        state_publisher = n.advertise<tuw_nav_msgs::ControllerState>("state_trajectory_ctrl", 10);
        pose_subscriber = n.subscribe("pose", 1000, &ControllerNode::onPoseReceived, this);
        command_subscriber = n.subscribe("ctrl", 1000, &ControllerNode::onCommandReceived, this);
        action_server.start();
    }

    void ControllerNode::onPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose){
        PathPoint pt;
        pt.x = pose->pose.pose.position.x;
        pt.y = pose->pose.pose.position.y;

        tf::Quaternion q(pose->pose.pose.orientation.x, pose->pose.pose.orientation.y, pose->pose.pose.orientation.z,
                         pose->pose.pose.orientation.w);
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
        geometry_msgs::Twist command;
        command.linear.x = v;
        command.angular.z = w;
        twist_publisher.publish(command);
    }

    void ControllerNode::publishControllerState(const nav_msgs::Path &path) {
        tuw_nav_msgs::ControllerState controller_state;
        controller_state.progress_in_relation_to = path.header.seq;
        controller_state.header.frame_id = path.header.frame_id;
        controller_state.header.stamp = ros::Time::now();
        controller_state.progress = getProgress();
        if (isActive()) {
            controller_state.state = controller_state.STATE_DRIVING;
        } else {
            controller_state.state = controller_state.STATE_IDLE;
        }
        state_publisher.publish(controller_state);
    }

    void ControllerNode::onCommandReceived(const std_msgs::String command){
        std::string s = command.data;

        ROS_INFO("Multi Robot Controller: received %s", s.c_str());
        if (s.compare("run") == 0) {
            setState(run);
        } else if (s.compare("stop") == 0) {
            setState(stop);
        } else if (s.compare("step") == 0) {
            setState(step);
        } else {
            setState(run);
        }
    }

    void ControllerNode::onGoalReceived(const tuw_local_controller_msgs::ExecutePathGoalConstPtr &goal) {

        const auto& path = goal->path;
        if (path.poses.empty()) {
            return;
        }

        ControllerConfig config;
        n_param_.getParam("max_v",config.max_v);
        n_param_.getParam("max_w", config.max_w);
        n_param_.getParam("goal_radius", config.goal_radius);
        n_param_.getParam("Kp", config.Kp);;
        n_param_.getParam("Ki", config.Ki);
        n_param_.getParam("Kd", config.Kd);
        setSpeedParams(config.max_v, config.max_w);
        setGoalRadius(config.goal_radius);
        setPID(config.Kp, config.Ki, config.Kd);

        setupController(path, config);

        tuw_local_controller_msgs::ExecutePathResult result;
        geometry_msgs::Pose pose;
        result.pose = pose;

        ros::Rate rate(5);
        int progress = getProgress();
        bool success = true;

        while (progress < goal->path.poses.size()) {
            if (action_server.isPreemptRequested() || !ros::ok()) {
                ROS_INFO("Goal preempted.");
                action_server.setPreempted(result);
                success = false;
                break;
            }

            tuw_local_controller_msgs::ExecutePathFeedback feedback;
            feedback.current_step = progress;
            action_server.publishFeedback(feedback);
            publishControllerState(path);
            rate.sleep();
            progress = getProgress();
        }

        if (success) {
            action_server.setSucceeded(result, "Robot finished path.");
        }
    }

    void ControllerNode::setupController(const nav_msgs::Path &path, const ControllerConfig& config) {

        auto it = path.poses.begin();
        std::vector<PathPoint> path_points;
        for (; it != path.poses.end(); ++it)
        {
            PathPoint pt;

            tf::Quaternion q(it->pose.orientation.x, it->pose.orientation.y, it->pose.orientation.z, it->pose.orientation.w);
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            pt.x = it->pose.position.x;
            pt.y = it->pose.position.y;
            pt.theta = yaw;

            path_points.push_back(pt);
        }

        setSpeedParams(config.max_v, config.max_w);
        setGoalRadius(config.goal_radius);
        setPID(config.Kp, config.Ki, config.Kd);
        setPath(std::make_shared<std::vector<PathPoint>>(path_points));
    }
}
