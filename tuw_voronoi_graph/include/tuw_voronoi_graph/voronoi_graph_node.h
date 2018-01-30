#ifndef VORONOI_GRAPH_NODE_H
#define VORONOI_GRAPH_NODE_H

#include <memory>

// ROS
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <tuw_voronoi_map/voronoi_path_generator.h>

namespace voronoi_graph
{


    class VoronoiGeneratorNode : public VoronoiPathGenerator
    {

        public:
            VoronoiGeneratorNode(ros::NodeHandle & n);

            ros::NodeHandle n_;
            ros::NodeHandle n_param_;
            std::unique_ptr<ros::Rate> rate_;
            void publishMap();

        private:
            ros::Publisher               pubVoronoiMap_;
            ros::Subscriber              subMap_;


            std::string                  topicGlobalMap_;
            std::string                  topicVoronoiMap_;

            std::string frameGlobalMap_;
            std::string frameVoronoiMap_;

            void globalMapCallback(const nav_msgs     ::OccupancyGrid            ::ConstPtr& _map);
            size_t current_map_hash_;
            size_t getHash(const std::vector<signed char> &_map, Eigen::Vector2d _origin, float _resolution);

            Eigen::Vector2d origin_;
            float resolution_;
            cv::Mat distField_;
            cv::Mat voronoiMap_;
    };

}

#endif // TUW_NAV_COSTMAP_NODE_H

