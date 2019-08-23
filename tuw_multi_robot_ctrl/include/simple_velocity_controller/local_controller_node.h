//
// Created by axelbr on 21.08.19.
//

#ifndef SRC_LOCAL_CONTROLLER_NODE_H
#define SRC_LOCAL_CONTROLLER_NODE_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tuw_nav_msgs/ControllerState.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <memory>
#include <actionlib/server/simple_action_server.h>
#include <tuw_local_controller_msgs/ExecutePathAction.h>
#include "controller.h"

namespace velocity_controller {

    struct ControllerConfig {
        float max_v = 0.8;
        float max_w = 1.0;
        float goal_radius = 0.2;
        float Kp = 5.0;
        float Ki = 0.0;
        float Kd = 1.0;
    };

    class ControllerNode: public velocity_controller::Controller {
        //special class-member functions.
    public:
        /**
         * @brief Construct a new Controller Node object
         *
         * @param n the node handle to subscribe topics
         */
        ControllerNode(ros::NodeHandle &n);

        ros::NodeHandle n_;       ///< Node handler to the root node
        ros::NodeHandle n_param_; ///< Node handler to the current node
        std::unique_ptr<ros::Rate> rate_;

    private:
        ros::Publisher twist_publisher;
        ros::Publisher state_publisher;
        ros::Subscriber pose_subscriber;
        ros::Subscriber command_subscriber;
        ros::Time last_update_;
        actionlib::SimpleActionServer<tuw_local_controller_msgs::ExecutePathAction> action_server;

        void onPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose);

        void onCommandReceived(const std_msgs::String command);

        void onGoalReceived(const tuw_local_controller_msgs::ExecutePathGoalConstPtr& goal);

        void publishControllerState(const nav_msgs::Path &path);

        void setupController(const nav_msgs::Path &path, const ControllerConfig& config);
    };

} // namespace velocity_controller

#endif //SRC_LOCAL_CONTROLLER_NODE_H
