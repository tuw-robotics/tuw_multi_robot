#ifndef VORONOI_GRAPH_NODE_H
#define VORONOI_GRAPH_NODE_H

#include <memory>

// ROS
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <tuw_voronoi_map/voronoi_path_generator.h>
#include <voronoi_segmentation/voronoi_graph_generator.h>

#include <tuw_voronoi_map/serializer.h>

namespace voronoi_graph
{


    class VoronoiGeneratorNode : public VoronoiPathGenerator, public VoronoiGraphGenerator, public Serializer
    {

        public:
            VoronoiGeneratorNode(ros::NodeHandle & n);

            ros::NodeHandle n_;
            ros::NodeHandle n_param_;
            std::unique_ptr<ros::Rate> rate_;
            void publishMap();
            void publishSegments();

        private:
            void globalMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& _map);
            size_t getHash(const std::vector<signed char> &_map, Eigen::Vector2d _origin, float _resolution);
            void createGraph(const nav_msgs::OccupancyGrid::ConstPtr& _map, size_t _map_hash);
            bool loadGraph(size_t hash);
            
            
            ros::Publisher                          pubVoronoiMap_;
            ros::Publisher                          pubSegments_;
            ros::Subscriber                         subMap_;
  

            std::string                             topicGlobalMap_;
            std::string                             topicVoronoiMap_;
            std::string                             topicSegments_;
            std::string                             graphPath_;
    
            std::string                             frameGlobalMap_;
            std::string                             frameVoronoiMap_;


            size_t                                  current_map_hash_;
            Eigen::Vector2d                         origin_;
            float                                   resolution_;
            cv::Mat                                 map_;
            cv::Mat                                 distField_;
            cv::Mat                                 voronoiMap_;
            float                                   path_length_;
            std::unique_ptr<float[]>                potential;
            std::vector<std::shared_ptr<Segment>>   segments_;
            int                                     smoothing_;
            float                                   crossingOptimization_;
            float                                   endSegmentOptimization_;
    };

}

#endif // TUW_NAV_COSTMAP_NODE_H

