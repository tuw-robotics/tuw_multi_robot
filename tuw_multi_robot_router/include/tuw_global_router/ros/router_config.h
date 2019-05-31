//
// Created by axelbr on 30.05.19.
//

#ifndef TUW_ROUTER_CONFIG_H
#define TUW_ROUTER_CONFIG_H

namespace
{
    struct RouterConfig
    {
        int planner_type;
        int collision_resolver;
        bool voronoi_graph;
        int goal_mode;


        int attempts_total;
        int attempts_successful;
        double sum_processing_time_total;
        double sum_processing_time_successful;
    };
}

#endif //SRC_ROUTER_CONFIG_H
