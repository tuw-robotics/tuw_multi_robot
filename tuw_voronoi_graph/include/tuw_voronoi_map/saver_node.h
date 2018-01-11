#ifndef TUW_VORONOI_SAVER_NODE_H
#define TUW_VORONOI_SAVER_NODE_H
    
#include <memory>

// ROS
#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <tuw_voronoi_map/serializer.h>

namespace voronoi_map {


class VoronoiSaverNode : public voronoi_map::VoronoiSerializer {

    //special class-member functions.
    public   : VoronoiSaverNode(ros::NodeHandle & n);
    
    //ROS:
    public   : ros::NodeHandle n_;      ///< Node handler to the root node
    public   : ros::NodeHandle n_param_;///< Node handler to the current node
    public   : std::unique_ptr<ros::Rate> rate_;
    
    // ROS Subscribers
    private  : ros::Subscriber              mapSubscriber_; 
    
    
    // ROS Topic names
    private  : std::string              topicVoronoiMap_;
    private  : std::string 		mapInfoName_;
    private  : std::string 		mapPath_;
    
    
    void mapCallback(const grid_map_msgs::GridMap& msg);
    
};

}

#endif // TUW_NAV_COSTMAP_NODE_H

