#include "ros/voronoi_graph_generator_node.h"

namespace tuw_graph
{
    VoronoiGraphGeneratorNode::VoronoiGraphGeneratorNode(ros::NodeHandle& node): node_(node), n_param_("~")
    {
        n_param_.param<bool>("publish_voronoi_map_image", publish_voronoi_map_image_, false);
        n_param_.param<double>("map_inflation", inflation_, 0.1);
        n_param_.param<float>("segment_length", segment_length_, 1.0);

        n_param_.param<float>("opt_crossings", crossing_optimization_, 0.2);
        n_param_.param<float>("opt_end_segments", end_segment_optimization_, 0.2);
        n_param_.param<std::string>("graph_cache_path", graph_cache_path_, "/tmp");
        n_param_.param<std::string>("custom_graph_path", custom_graph_path_, "");

        if (graph_cache_path_.back() != '/')
        {
            graph_cache_path_ += "/";
        }
        
        if (custom_graph_path_.back() != '/' && !custom_graph_path_.empty())
        {
            custom_graph_path_ += "/";
        }

        map_subscriber_ = node.subscribe("map", 1, &VoronoiGraphGeneratorNode::mapCallback, this);
        graph_publisher_ = node.advertise<tuw_multi_robot_msgs::Graph>("segments", 1);
        if (publish_voronoi_map_image_)
        {
            voronoi_map_publisher_ = node.advertise<nav_msgs::OccupancyGrid>("map_eroded", 1);
        }
    }

    void VoronoiGraphGeneratorNode::publishMap()
    {
        ROS_INFO("Hi!");
    }

    void VoronoiGraphGeneratorNode::publishSegments()
    {
    }

    void VoronoiGraphGeneratorNode::mapCallback(const nav_msgs::OccupancyGridConstPtr& map)
    {
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "voronoi_graph_generator_node"); /// initializes the ros node with default name
    ros::NodeHandle n;
    tuw_graph::VoronoiGraphGeneratorNode voronoi_graph_generator_node(n);
    ros::Rate rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        voronoi_graph_generator_node.publishSegments();
        rate.sleep();
    }
}

