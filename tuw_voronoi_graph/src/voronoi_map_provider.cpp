#include <tuw_voronoi_map/voronoi_map_provider.h>
#include <ros/ros.h>
#include <memory>


namespace voronoi_map {
  
void voronoi_map::VoronoiMapProvider::mapCallback(const grid_map_msgs::GridMap& msg)
{
  grid_map::GridMap voronoiMap;
  grid_map::GridMapRosConverter::fromMessage(msg, voronoiMap);
  
  if(!got_map_);
  {
    map_ = std::make_shared<grid_map::GridMap>(voronoiMap);
  }
  got_map_ = true;
}

voronoi_map::VoronoiMapProvider::VoronoiMapProvider(std::string topicName)
{
  //Initialize subscriber
  ros::NodeHandle n;
  mapSubscriber_ = n.subscribe(topicName, 1, &VoronoiMapProvider::mapCallback, this); 
  
}

std::shared_ptr<grid_map::GridMap> voronoi_map::VoronoiMapProvider::get_grid_map()
{
  return std::shared_ptr<grid_map::GridMap>(map_);
}

bool voronoi_map::VoronoiMapProvider::map_available()
{
  return got_map_;
}

}