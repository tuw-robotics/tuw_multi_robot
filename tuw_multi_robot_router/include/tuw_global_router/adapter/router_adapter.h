//
// Created by axelbr on 05.06.19.
//

#ifndef TUW_MULTI_ROBOT_ROUTER_ROUTER_ADAPTER_H
#define TUW_MULTI_ROBOT_ROUTER_ROUTER_ADAPTER_H

#include <tuw_global_router/router/router.h>
#include <tuw_global_router/legacy/router.h>
#include <tuw_multi_robot_router/routerConfig.h>

class RouterAdapter: public tuw::multi_robot_router::Router
{
public:
    explicit RouterAdapter(multi_robot_router::Router &router);

    void setConfig(const tuw_multi_robot_router::routerConfig &config);

    void setResolution(float resolution);

    void setMap(cv::Mat& map);

    void setOrigin(Eigen::Vector2d origin);

    tuw::multi_robot_router::RoutingTable computePlan(const std::vector<tuw::multi_robot_router::Robot> &robots, const tuw::multi_robot_router::Graph& graph) override;

private:
    multi_robot_router::Router &router_;

    cv::Mat map_;
    float resolution_;
    Eigen::Vector2d origin_;
};


#endif //TUW_MULTI_ROBOT_ROUTER_ROUTER_ADAPTER_H
