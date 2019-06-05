#include <tuw_global_router/adapter/router_adapter.h>

RouterAdapter::RouterAdapter(multi_robot_router::Router &router): router_(router)
{

}

void RouterAdapter::setResolution(float resolution)
{
    resolution_ = resolution;
}

void RouterAdapter::setMap(cv::Mat &map)
{
    map_ = map;
}

void RouterAdapter::setConfig(const tuw_multi_robot_router::routerConfig &config)
{
    //Important set router before settings
    uint32_t threads = config.nr_threads;
    if (config.router_type == 1)
        router_.setPlannerType(multi_robot_router::Router::routerType::multiThreadSrr, threads);
    else
        router_.setPlannerType(multi_robot_router::Router::routerType::singleThread, 1);

    if (config.collision_resolver == 0)
    {
        router_.setCollisionResolutionType(multi_robot_router::SegmentExpander::CollisionResolverType::none);
        router_.collisionResolver_ = false;
    } else if (config.collision_resolver == 1)
    {
        router_.setCollisionResolutionType(multi_robot_router::SegmentExpander::CollisionResolverType::backtracking);
        router_.collisionResolver_ = true;
    } else
    {
        router_.setCollisionResolutionType(multi_robot_router::SegmentExpander::CollisionResolverType::avoidance);
        router_.collisionResolver_ = true;
    }

    if (config.voronoi_graph)
        router_.graphMode_ = multi_robot_router::Router::graphType::voronoi;
    else
        router_.graphMode_ = multi_robot_router::Router::graphType::random;

    if (config.goal_mode == 0)
        router_.goalMode_ = multi_robot_router::Router::goalMode::use_map_goal;
    else if (config.goal_mode == 1)
        router_.goalMode_ = multi_robot_router::Router::goalMode::use_voronoi_goal;
    else
        router_.goalMode_ = multi_robot_router::Router::goalMode::use_segment_goal;

    router_.routerTimeLimit_s_ = config.router_time_limit_s;


    router_.priorityRescheduling_ = config.priority_rescheduling;
    router_.speedRescheduling_ = config.speed_rescheduling;
    router_.segmentOptimizations_ = config.path_endpoint_optimization;
}

void RouterAdapter::setOrigin(Eigen::Vector2d origin)
{
    origin_ = origin;
}

tuw::multi_robot_router::RoutingTable
RouterAdapter::computePlan(const std::vector<tuw::multi_robot_router::Robot> &robots,
                           const tuw::multi_robot_router::Graph &graph)
{
    router_.makePlan()
    return tuw::multi_robot_router::RoutingTable();
}
