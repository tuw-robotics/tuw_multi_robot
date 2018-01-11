#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_broadcaster.h>
#include <tuw_voronoi_map/publisher_node.h>
#include <memory>
#include <grid_map_ros/grid_map_ros.hpp>



int main(int argc, char** argv) {

    ros::init ( argc, argv, "voronoi_serializer" );  /// initializes the ros node with default name
    ros::NodeHandle n; 
    
    voronoi_map::VoronoiPublisherNode mapNode ( n );
    ros::Rate r(1);
    
    while ( ros::ok()  ) 
    {
      mapNode.publishGridMap();
      r.sleep();
      ros::spinOnce();
    }
    
    return 0;
}


namespace voronoi_map {
  
void VoronoiPublisherNode::publishGridMap()
{   
    static grid_map_msgs::GridMap msg;
    
    static std::vector<std::string> pubLayers    = { "map" , "distfield" , "voronoi"};
    grid_map::GridMapRosConverter::toMessage(map_   , pubLayers   , msg); 
    
    pubVoronoiMap_.publish(msg);   
}

VoronoiPublisherNode::VoronoiPublisherNode ( ros::NodeHandle & n ) :  voronoi_map::VoronoiSerializer (  ), 
    n_ ( n ), 
    n_param_ ( "~" ) {
    
    topicVoronoiMap_ = "voronoi_map";
    n_param_.param("map_topic", topicVoronoiMap_, topicVoronoiMap_);
    
    mapInfoName_ = "vmap.info";
    n_param_.param("mapInfoName", mapInfoName_, mapInfoName_);
    
    mapPath_ = "./";
    n_param_.param("mapPath", mapPath_, mapInfoName_);
        
    
    loadMap(mapInfoName_, mapPath_);  
    
    VoronoiMapInfo info = getMapInfo();
    ROS_INFO("Publishing map (%s):  size: [%i, %i]; resolution: %f; origin: [%f, %f]", mapInfoName_.c_str(), info.cols, info.rows, info.resolution, info.origin_cols, info.origin_rows);
    
    
    pubVoronoiMap_    = n.advertise<grid_map_msgs  ::GridMap     >(topicVoronoiMap_   , 1/*, true*/);
    
     
    
    ros::Rate(1).sleep();
    ros::spinOnce();
}

}