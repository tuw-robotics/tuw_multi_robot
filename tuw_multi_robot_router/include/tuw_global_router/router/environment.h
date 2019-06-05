//
// Created by axelbr on 30.05.19.
//

#ifndef TUW_ENVIRONMENT_H
#define TUW_ENVIRONMENT_H

#include <opencv2/opencv.hpp>
#include <tuw_global_router/legacy/srr_utils.h>
#include <eigen3/Eigen/Eigen>

namespace multi_robot_router
{
    struct Environment
    {
        cv::Mat map;
        float resolution;
        std::vector<Segment> graph;
        Eigen::Vector2d origin;
    };
}

#endif //SRC_ENVIRONMENT_H
