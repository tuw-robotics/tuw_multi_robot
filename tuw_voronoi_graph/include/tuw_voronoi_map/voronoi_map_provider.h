#ifndef _VORONOI_MAP_ROS
#define _VORONOI_MAP_ROS

#include <nav_msgs/OccupancyGrid.h>
#include <memory>
#include <opencv/cv.h>
#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>

#define DEFAULT_MAP_NAME	"voronoi_map"

namespace voronoi_map {
  class VoronoiMapProvider
  {
  public:
      VoronoiMapProvider(std::string topicName);
      std::shared_ptr<grid_map::GridMap> get_grid_map();
      bool map_available();
  private:
    void mapCallback(const grid_map_msgs::GridMap& msg);
    std::shared_ptr<grid_map::GridMap> map_;
    bool got_map_ = false;
    ros::Subscriber mapSubscriber_;    
  };
}
#endif