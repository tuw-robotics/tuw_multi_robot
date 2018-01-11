#ifndef TUW_VORONOI_PUBLISHER_NODE_H
#define TUW_VORONOI_PUBLISHER_NODE_H
    
#include <memory>

// ROS
#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <tuw_voronoi_map/serializer.h>

namespace voronoi_map {


class VoronoiPublisherNode : public voronoi_map::VoronoiSerializer {

    //special class-member functions.
    public   : VoronoiPublisherNode(ros::NodeHandle & n);
    
    //ROS:
    public   : ros::NodeHandle n_;      ///< Node handler to the root node
    public   : ros::NodeHandle n_param_;///< Node handler to the current node
    public   : std::unique_ptr<ros::Rate> rate_;
    public   : void publishGridMap();
    
    // ROS Subscribers
    private  : ros::Publisher              pubVoronoiMap_; 
    
    
    // ROS Topic names
    private  : std::string              topicVoronoiMap_;
    private  : std::string 		mapInfoName_;
    private  : std::string 		mapPath_;
    
    
};

}

#endif // TUW_NAV_COSTMAP_NODE_H

