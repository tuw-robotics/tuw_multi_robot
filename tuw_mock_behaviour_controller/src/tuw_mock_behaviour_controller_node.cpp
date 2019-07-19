//
// Created by axelbr on 15.07.19.
//

#include <ros/node_handle.h>
#include <tuw_multi_robot_msgs/BehaviourProfile.h>
#include <tuw_multi_robot_msgs/RobotInfo.h>
#include <tuw_multi_robot_msgs/Route.h>
#include <unordered_map>


void printCurrentSegments(const std::unordered_map<std::string, int> &segments,
                          const tuw_multi_robot_msgs::BehaviourProfile &profile);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mock_behaviour_controller");
    ros::NodeHandle node_handle;

    std::unordered_map<std::string, ros::Subscriber> route_subscribers;
    std::unordered_map<std::string, ros::Publisher> behaviour_publishers;
    std::unordered_map<std::string, std::unordered_map<int, tuw_multi_robot_msgs::BehaviourProfile>> route_to_profiles_map;
    std::unordered_map<std::string, int> current_segments;


    auto robot_info_callback = [&](const tuw_multi_robot_msgs::RobotInfoConstPtr &robot_info) {
        if (route_subscribers.find(robot_info->robot_name) == route_subscribers.end()) {
            ROS_INFO("No robot with name %s found. Setup entry.", robot_info->robot_name.c_str());

            ros::Publisher behaviour_publisher = node_handle.advertise<tuw_multi_robot_msgs::BehaviourProfile>(
                    "/" + robot_info->robot_name + "/behaviour", 10);

            behaviour_publishers[robot_info->robot_name] = behaviour_publisher;

            std::string name = robot_info->robot_name;
            auto route_callback = [name, &route_to_profiles_map](const tuw_multi_robot_msgs::RouteConstPtr &route) {
                ROS_INFO("Received route message from %s", name.c_str());
                int id = 0;
                for (const auto &segment : route->segments) {
                    tuw_multi_robot_msgs::BehaviourProfile profile;
                    profile.name = "slow - " + name + " - " + std::to_string(id);
                    profile.description = "slfkajk sjlfk sklj skjl";
                    route_to_profiles_map[name][id] = profile;
                    id++;
                }
            };

            route_subscribers[robot_info->robot_name] = node_handle.subscribe<tuw_multi_robot_msgs::Route>(
                    "/" + robot_info->robot_name + "/route", 10, route_callback);

        } else {
            current_segments[robot_info->robot_name] = robot_info->sync.current_route_segment;
        }

    };

    ros::Subscriber robot_info_subscriber = node_handle.subscribe<tuw_multi_robot_msgs::RobotInfo>("/robot_info", 10,
                                                                                                   robot_info_callback);

    ros::Rate rate(1);
    while (ros::ok()) {

        ros::spinOnce();

        for (const auto &tuple : behaviour_publishers) {
            const auto &name = tuple.first;
            const auto &publisher = tuple.second;
            int current_segment = current_segments[name];
            const auto &profile = route_to_profiles_map[name][current_segment];
            behaviour_publishers[name].publish(profile);
            printCurrentSegments(current_segments, profile);
        }

        rate.sleep();
    }
}


void printCurrentSegments(const std::unordered_map<std::string, int> &segments,
                          const tuw_multi_robot_msgs::BehaviourProfile &profile)
{
    std::string message;
    std::for_each(segments.begin(), segments.end(), [&](const std::pair<std::string, int> &tuple) {
        // const auto &profile = route_to_profiles_map[name][current_segment];
        message += "[ robot: " + tuple.first + ", segment: " + std::to_string(tuple.second) + ", profile: " +
                   profile.name + "]\n";
    });
    ROS_INFO("\n%s", message.c_str());
}
