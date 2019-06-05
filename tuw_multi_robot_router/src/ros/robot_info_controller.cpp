#include <tuw_global_router/ros/robot_info_controller.h>

namespace multi_robot_router
{
    float RobotInfoController::maxRadius() const
    {
        return max_radius_;
    }

    optional<models::Robot> RobotInfoController::findRobot(const string &name)
    {
        RobotInfo* info = findByName(name);
        if (info == nullptr)
        {
            return nullopt;
        } else
        {
            models::Robot robot{info->robot_name, info->radius()};
            return make_optional(robot);
        }
    }


    vector<models::Robot> RobotInfoController::robots() const
    {
        vector<models::Robot> robots;
        transform(subscribed_robots_.begin(), subscribed_robots_.end(), back_inserter(robots), [](const auto& entry) {
            models::Robot r{entry.second.robot_name, entry.second.radius()};
            return r;
        });
        return robots;
    }

    RobotInfo* RobotInfoController::findByName(const string &name){
        auto subscribed_robot = subscribed_robots_.find(name);
        if (subscribed_robot == subscribed_robots_.end()) {
            return nullptr;
        } else {
            return &subscribed_robot->second;
        }
    }

    void RobotInfoController::processRobotInfo(const tuw_multi_robot_msgs::RobotInfo &robot_info){
        auto subscribed_robot = findByName(robot_info.robot_name);

        if (subscribed_robot != nullptr) {
           subscribed_robot->updateInfo(robot_info);
        } else {
            RobotInfo new_robot{robot_info};
            new_robot.initTopics(node_handle_, !single_robot_mode_);
            subscribed_robots_.emplace(make_pair(new_robot.robot_name, new_robot));
            max_radius_ = max(new_robot.radius(), max_radius_);
        }

        if (single_robot_mode_ && (subscribed_robots_.size() > 1)) {
            ROS_WARN("More than one robot subscribed, but the MRRP is in single robot mode");
        }
    }
}