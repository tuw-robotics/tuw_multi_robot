#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_broadcaster.h>
#include <tuw_voronoi_map/saver_node.h>
#include <memory>
#include <grid_map_ros/grid_map_ros.hpp>


bool got_map_ = false;

int main(int argc, char** argv) {

    ros::init ( argc, argv, "voronoi_serializer" );  /// initializes the ros node with default name
    ros::NodeHandle n; 
    
    voronoi_map::VoronoiSaverNode mapNode ( n );
    
    while ( ros::ok() && !got_map_ ) 
    {
      ros::spinOnce();
    }
    
    
    return 0;
}


namespace voronoi_map {
  
  
void VoronoiSaverNode::mapCallback(const grid_map_msgs::GridMap& msg)
{
  ROS_INFO("saving map");
  if(!got_map_);
  {
    grid_map::GridMapRosConverter::fromMessage(msg, map_);
    saveMap(mapInfoName_, mapPath_);
  }
  
  VoronoiMapInfo info = getMapInfo();
  ROS_INFO("Saved map (%s):  size: [%i, %i]; resolution: %f; origin: [%f, %f]", mapInfoName_.c_str(), info.cols, info.rows, info.resolution, info.origin_cols, info.origin_rows);
    
  got_map_ = true;
}

VoronoiSaverNode::VoronoiSaverNode ( ros::NodeHandle & n ) :  voronoi_map::VoronoiSerializer (  ), 
    n_ ( n ), 
    n_param_ ( "~" ) {
    
    topicVoronoiMap_ = "voronoi_map";
    n_param_.param("map_topic", topicVoronoiMap_, topicVoronoiMap_);
    
    
    mapInfoName_ = "vmap.info";
    n_param_.param("mapInfoName", mapInfoName_, mapInfoName_);
    
    mapPath_ = "./";
    n_param_.param("mapPath", mapPath_, mapPath_);
    ROS_INFO("%s", mapPath_.c_str());
    
    mapSubscriber_ = n.subscribe(topicVoronoiMap_, 1, &VoronoiSaverNode::mapCallback, this); 
    
    ros::Rate(1).sleep();
    ros::spinOnce();
}

}
