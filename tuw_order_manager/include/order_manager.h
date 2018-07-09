#ifndef __TUW_ORDERMANAGER_H
#define __TUW_ORDERMANAGER_H

#include <stdio.h>

#include <ros/ros.h>
#include <tuw_multi_robot_msgs/Goods.h>
#include <tuw_multi_robot_msgs/GoodPosition.h>
#include <tuw_multi_robot_msgs/Pickup.h>
#include <tuw_multi_robot_msgs/RobotInfo.h>
#include <nav_msgs/Odometry.h>
#include <tuw_geometry_msgs/pose.h>
#include <map>

#include "abstractsolver.h"
#include "simplesolver.h"

namespace tuw_order_manager {

enum modes {
    MODE_INIT,
    MODE_PROGRESS
};

class OrderManager
{
public:
    OrderManager(int, char**);
    void run();
private:
    ros::Publisher pub_robot_goals;
    ros::Publisher pub_pickup;
    ros::Publisher pub_good_pose;
    std::vector<ros::Subscriber> subscribers;
    ros::NodeHandle *nodeHandle;

    void robotInfoCallback(const tuw_multi_robot_msgs::RobotInfo::ConstPtr& robotInfo);
    void goodsCallback(const tuw_multi_robot_msgs::Goods::ConstPtr& goods);
    void odomCallback(const nav_msgs::Odometry& odom);
    int increment_robot_progress(std::string robot_name);
    void subscribe_robot_odom();
    void route();
    void reset();

    std::map<std::string, tuw::ros_msgs::Pose*> subscribed_robots;
    std::map<std::string, int> robots_status;
    std::map<std::string, int> robots_progress;
    std::vector<transport_pair> transport_pairs;
    std::map<std::string, int> attached_goods;

    std::vector<tuw_multi_robot_msgs::Good> goods;

    int mode;

    tuw_multi_robot_msgs::Good *findGoodByRobotName(std::string robot_name);
    tuw_multi_robot_msgs::Good *findGoodByGoodId(int id);
    int findGoodIdByRobotName(std::string robot_name);

    void publishGoodPosition(int good_id, geometry_msgs::Pose pose);
    void publishPickup(std::string robot_name, int good_id);
};

} // end namespace tuw_order_manager

#endif
