//
// Created by axelbr on 05.06.19.
//

#ifndef TUW_MULTI_ROBOT_ROUTER_ROUTER_H
#define TUW_MULTI_ROBOT_ROUTER_ROUTER_H

#include <tuw_global_router/router/models.h>
#include <vector>

namespace tuw::multi_robot_router
{
    class Router
    {
        virtual RoutingTable computePlan(const std::vector<Robot>& robots, const Graph& environment) = 0;
    };
}

#endif //TUW_MULTI_ROBOT_ROUTER_ROUTER_H
