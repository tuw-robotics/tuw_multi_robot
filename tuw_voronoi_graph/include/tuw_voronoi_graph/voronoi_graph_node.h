#ifndef VORONOI_GRAPH_NODE_H
#define VORONOI_GRAPH_NODE_H

#include <memory>

// ROS
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <tuw_voronoi_map/voronoi_path_generator.h>
#include <tuw_voronoi_graph/voronoi_graph_generator.h>

#include <tuw_serialization/serializer.h>

namespace tuw_graph
{


    class VoronoiGeneratorNode : public voronoi_map::VoronoiPathGenerator, public VoronoiGraphGenerator, public Serializer
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
            void createGraph(const nav_msgs::OccupancyGrid::ConstPtr& _map, size_t _map_hash);
            bool loadGraph(size_t _hash);
            bool loadCustomGraph(std::string _path);
            
            
            ros::Publisher                          pubVoronoiMapImage_;
            ros::Publisher                          pubSegments_;
            ros::Subscriber                         subMap_;
  

            std::string                             graphCachePath_;
            std::string                             customGraphPath_;
    
            std::string                             frameGlobalMap_;
            std::string                             frameVoronoiMap_;


            size_t                                  current_map_hash_;
            bool                                    publishVoronoiMapImage_;    /// for debuging
            Eigen::Vector2d                         origin_;
            float                                   resolution_;
            cv::Mat                                 map_;
            cv::Mat                                 distField_;
            cv::Mat                                 voronoiMap_;
            float                                   segment_length_;
            std::unique_ptr<float[]>                potential;
            std::vector<Segment>                    segments_;
            int                                     smoothing_;
            double                                  inflation_;
            float                                   crossingOptimization_;
            float                                   endSegmentOptimization_;
            
            
            nav_msgs::OccupancyGrid voronoiMapImage_;
    };

}

#endif // TUW_NAV_COSTMAP_NODE_H

