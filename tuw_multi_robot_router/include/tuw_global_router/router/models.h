#include <utility>

//
// Created by axelbr on 05.06.19.
//

#ifndef TUW_MULTI_ROBOT_ROUTER_MODELS_H
#define TUW_MULTI_ROBOT_ROUTER_MODELS_H

#include <string>
#include <vector>
#include <optional>

namespace tuw::multi_robot_router
{
    struct Position
    {
        double x;
        double y;
        double z;
    };

    struct Orientation
    {
        double roll;
        double pitch;
        double yaw;
    };

    struct Pose
    {
        Position position;
        Orientation orientation;
    };

    struct Robot
    {
        std::optional<Pose> start;
        std::optional<Pose> goal;
        double radius;
        std::string name;

        Robot(std::string name, double radius): name(std::move(name)), radius(radius)
        {
        }

        Robot(const Robot& robot) = default;
    };
    struct Segment
    {
        int id;
        Pose start;
        Pose end;
        double width;
    };

    struct RoutingTable
    {

    };

    struct Route
    {

    };

    struct Graph
    {};
}

#endif //TUW_MULTI_ROBOT_ROUTER_MODELS_H
