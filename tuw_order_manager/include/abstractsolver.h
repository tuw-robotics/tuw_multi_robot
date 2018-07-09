#ifndef __TUW_ORDERMANAGER_ABSTRACTSOLVER_H
#define __TUW_ORDERMANAGER_ABSTRACTSOLVER_H

#include <ros/ros.h>
#include <tuw_multi_robot_msgs/Good.h>
#include <nav_msgs/Odometry.h>
#include <tuw_geometry_msgs/pose.h>
#include <map>
#include <string>

namespace tuw_order_manager {

struct transport_pair {
    std::string robot_name;
    int good_id;
};

class AbstractSolver
{
public:
    explicit AbstractSolver(std::map<std::string, tuw::ros_msgs::Pose*> robots, std::vector<tuw_multi_robot_msgs::Good> goods) : robots(robots), goods(goods){};
    virtual std::vector<transport_pair> solve() = 0;
protected:
    std::map<std::string, tuw::ros_msgs::Pose*> robots;
    std::vector<tuw_multi_robot_msgs::Good> goods;
};

} // end namespace tuw_order_manager

#endif
