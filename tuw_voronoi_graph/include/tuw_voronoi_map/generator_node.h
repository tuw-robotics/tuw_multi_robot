#ifndef GENERATOR_NODE_H
#define GENERATOR_NODE_H
    
#include <memory>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <tuw_voronoi_map/generator.h>

namespace  voronoi_map{


class VoronoiGeneratorNode : public voronoi_map::VoronoiGenerator {

    //special class-member functions.
    public   : VoronoiGeneratorNode(ros::NodeHandle & n);
    
    //ROS:
    public   : ros::NodeHandle n_;      ///< Node handler to the root node
    public   : ros::NodeHandle n_param_;///< Node handler to the current node
    public   : std::unique_ptr<ros::Rate> rate_;
    
    // ROS Publishers
    private  : ros::Publisher  pubVoronoiMap_;
    
    // ROS Publisher functions
    public   : void publishGridMap  ();
    
    // ROS Subscribers
    private  : ros::Subscriber              subMap_; 
    
    
    // ROS tf
    private  : tf::TransformListener    tf_listener_;
    private  : tf::TransformBroadcaster tf_broadcaster_;
    
    // ROS Topic names
    private  : std::string              topicGlobalMap_;
    private  : std::string              topicVoronoiMap_;
    
    // Frames / links names
    private  : std::string frameGlobalMap_;
    private  : std::string frameVoronoiMap_;
    
    private  : void globalMapCallback          ( const nav_msgs     ::OccupancyGrid            ::ConstPtr& _map );
    
    grid_map::GridMap voronoiMap_;
};

}

#endif // TUW_NAV_COSTMAP_NODE_H

