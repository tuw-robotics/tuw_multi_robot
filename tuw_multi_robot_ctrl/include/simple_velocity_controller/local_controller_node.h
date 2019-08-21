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
#include "controller.h"
#include "../../../../../devel/include/tuw_local_controller_msgs/ExecutePathGoal.h"
#include <tuw_local_controller_util/LocalControllerService.h>

namespace velocity_controller {
    class ControllerNode
            : public velocity_controller::Controller, public tuw_local_controller::util::LocalControllerService {
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

        void onGoalReceived(const tuw_local_controller_msgs::ExecutePathGoal &goal) override;

    private:
        ros::Publisher pubCmdVel_;
        ros::Publisher pubState_;
        ros::Subscriber subPose_;
        ros::Subscriber subPath_;
        ros::Subscriber subCtrl_;
        float max_vel_v_;
        float max_vel_w_;
        float goal_r_;
        float Kp_val_;
        float Ki_val_;
        float Kd_val_;
        ros::Time last_update_;

        void subPoseCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &_pose);

        void subCtrlCb(const std_msgs::String _cmd);

        void publishState();

        geometry_msgs::Twist cmd_;
        tuw_nav_msgs::ControllerState ctrl_state_;
        geometry_msgs::Pose current_pose;
    };

} // namespace velocity_controller

#endif //SRC_LOCAL_CONTROLLER_NODE_H
