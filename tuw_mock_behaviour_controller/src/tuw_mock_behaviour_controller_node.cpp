//
// Created by axelbr on 15.07.19.
//

#include <ros/node_handle.h>
#include <tuw_multi_robot_msgs/BehaviourProfile.h>
#include <tuw_multi_robot_msgs/RobotInfo.h>
#include <tuw_multi_robot_msgs/Route.h>
#include <unordered_map>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mock_behaviour_controller");
    ros::NodeHandle node_handle;

    std::unordered_map<std::string, ros::Subscriber> route_subscribers;
    std::unordered_map<std::string, ros::Publisher> behaviour_publishers;
    std::unordered_map<std::string, int> current_segments;


    auto robot_info_callback = [&](const tuw_multi_robot_msgs::RobotInfoConstPtr &robot_info) {
        if (route_subscribers.find(robot_info->robot_name) == route_subscribers.end()) {
            ros::Publisher behaviour_publisher = node_handle.advertise<tuw_multi_robot_msgs::BehaviourProfile>(
                    "/" + robot_info->robot_name + "/behaviour", 10);
            auto route_callback = [&](const tuw_multi_robot_msgs::RouteConstPtr &route) {
                int current_segment = current_segments[robot_info->robot_name];
                if (current_segment >= 0) {
                    auto &behaviour = route->profiles[current_segment];
                    behaviour_publishers[robot_info->robot_name].publish(behaviour);
                }
            };

            route_subscribers[robot_info->robot_name] = node_handle.subscribe<tuw_multi_robot_msgs::Route>(
                    "/" + robot_info->robot_name + "/route", 10, route_callback);
        }
        current_segments[robot_info->robot_name] = robot_info->sync.current_route_segment;
    };

    ros::Subscriber robot_info_subscriber = node_handle.subscribe<tuw_multi_robot_msgs::RobotInfo>("/robot_info", 10,
                                                                                                   robot_info_callback);

    ros::spin();
}