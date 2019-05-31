//
// Created by axelbr on 30.05.19.
//

#ifndef TUW_AGENT_H
#define TUW_AGENT_H

#include <string>

namespace multi_robot_router
{
    struct Position
    {
        double x;
        double y;
        double z;
    };

    struct Agent
    {
        Position start;
        Position goal;
        double radius;
        std::string name;
    };
}

#endif //SRC_AGENT_H
