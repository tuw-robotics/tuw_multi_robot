#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_broadcaster.h>
#include <tuw_voronoi_map/generator_node.h>
#include <memory>

voronoi_map::VoronoiGenerator *map_;

void publishTf(std::string mapName);

bool allowPublish = false;

int main(int argc, char** argv) {

    ros::init ( argc, argv, "euclidean_distance_calculator" );  /// initializes the ros node with default name
    ros::NodeHandle n; 
    
    voronoi_map::VoronoiGeneratorNode mapNode ( n );
    
    ros::Rate r(1);
    
    ROS_INFO("Initialization done!");
    while ( ros::ok() ) 
    {
      ros::spinOnce();
      if(allowPublish)
        mapNode.publishGridMap();
      r.sleep();
    }
    return 0;
}


namespace voronoi_map {
/**
 * Constructor
 **/

VoronoiGeneratorNode::VoronoiGeneratorNode ( ros::NodeHandle & n ) :  voronoi_map::VoronoiGenerator (  ), 
    n_ ( n ), 
    n_param_ ( "~" ) {
    
    topicGlobalMap_ = "/map";
    n.param("map_topic", topicGlobalMap_, topicGlobalMap_);
    
    topicVoronoiMap_= "voronoi_map";
    n.param("voronoi_topic", topicVoronoiMap_, topicVoronoiMap_);
    
    frameGlobalMap_ = "map";
    n.param("map_frame", frameGlobalMap_, frameGlobalMap_);
    frameVoronoiMap_ = "voronoi_map";
    n.param("voronoi_frame", frameVoronoiMap_, frameVoronoiMap_);
     
    subMap_       = n.subscribe( topicGlobalMap_, 1, &VoronoiGeneratorNode::globalMapCallback, this );
    pubVoronoiMap_    = n.advertise<grid_map_msgs  ::GridMap     >(topicVoronoiMap_   , 1/*, true*/);
    
    //ros::Rate(1).sleep();
    ros::spinOnce();
    
    voronoiMap_.setFrameId("map");
}

void VoronoiGeneratorNode::globalMapCallback ( const nav_msgs::OccupancyGrid::ConstPtr& _map ) {
	grid_map::GridMapRosConverter::fromOccupancyGrid(*_map, "map", voronoiMap_);
	ROS_INFO("Created global map with size %f x %f m (%i x %i cells).\n The center of the map is located at (%f, %f) in the %s frame.",
			voronoiMap_.getLength().x(), voronoiMap_.getLength().y(),
			voronoiMap_.getSize()(0), voronoiMap_.getSize()(1),
			voronoiMap_.getPosition().x(), voronoiMap_.getPosition().y(), voronoiMap_.getFrameId().c_str());
	
	computeDistanceField(voronoiMap_,"map", "distfield");
	computeVoronoiMap(voronoiMap_,"distfield","voronoi");
	
	allowPublish = true;
}

void VoronoiGeneratorNode::publishGridMap()
{   
    static grid_map_msgs::GridMap msg;
    
    static std::vector<std::string> pubLayers    = { "map" , "distfield" , "voronoi"};
    grid_map::GridMapRosConverter::toMessage(voronoiMap_   , pubLayers   , msg); 
    
    pubVoronoiMap_.publish(msg);    
}

}