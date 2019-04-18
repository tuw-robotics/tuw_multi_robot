#ifndef VORONOI_GRAPH_GENERATOR_NODE_H
#define VORONOI_GRAPH_GENERATOR_NODE_H

#include <ros/ros.h>
#include <tuw_voronoi_map/voronoi_path_generator.h>
#include <tuw_voronoi_graph/voronoi_graph_generator.h>
#include <tuw_serialization/serializer.h>
#include <tuw_multi_robot_msgs/Graph.h>

namespace tuw_graph {
    class VoronoiGraphGeneratorNode {
    public:
        explicit VoronoiGraphGeneratorNode(ros::NodeHandle &node);

        void publishMap();

        void publishSegments();

        const bool &publish_voronoi_map_image() const {
            return publish_voronoi_map_image_;
        }

        const double &inflation() const {
            return inflation_;
        }

        const float &segment_length() const {
            return segment_length_;
        }

        const float &crossing_optimization() const {
            return crossing_optimization_;
        }

        const float &end_segment_optimization() const {
            return end_segment_optimization_;
        }

        const std::string &graph_cache_path() const {
            return graph_cache_path_;
        }

        const std::string &custom_graph_path() const {
            return custom_graph_path_;
        }

    private:
        ros::NodeHandle node_;
        ros::NodeHandle n_param_;
        ros::Subscriber map_subscriber_;
        ros::Publisher graph_publisher_;
        ros::Publisher voronoi_map_publisher_;

        bool publish_voronoi_map_image_;
        double inflation_;
        float segment_length_;
        float crossing_optimization_;
        float end_segment_optimization_;
        std::string graph_cache_path_;
        std::string custom_graph_path_;

        void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map);
    };
}

#endif