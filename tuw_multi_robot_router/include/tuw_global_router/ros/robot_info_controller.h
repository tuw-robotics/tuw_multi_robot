//
// Created by axelbr on 31.05.19.
//

#ifndef TUW_MULTI_ROBOT_ROUTER_ROBOT_INFO_CONTROLLER_H
#define TUW_MULTI_ROBOT_ROUTER_ROBOT_INFO_CONTROLLER_H

#include <ros/ros.h>
#include <vector>
#include <unordered_map>
#include <optional>
#include <tuw_global_router/legacy/robot_info.h>
#include <tuw_global_router/router/models.h>

using namespace std;

namespace multi_robot_router
{
    class RobotInfoController
    {
    public:
        void processRobotInfo(const tuw_multi_robot_msgs::RobotInfo &robot_info);
        float maxRadius() const;

        optional<models::Robot> findRobot(const string& name);
        vector<models::Robot> robots() const;
        RobotInfo* findByName(const string& name);

    private:
        ros::NodeHandle& node_handle_;
        const ros::NodeHandle param_handle_;
        ros::Subscriber robot_info_subscriber_;

        bool single_robot_mode_;
        float max_radius_;
        unordered_map<string, RobotInfo> subscribed_robots_;


    };
}

#endif //TUW_MULTI_ROBOT_ROUTER_ROBOT_INFO_CONTROLLER_H
