
#ifndef _GENERATOR
#define _GENERATOR

#include <nav_msgs/OccupancyGrid.h>
#include <memory>
#include <opencv/cv.h>
#include <grid_map_ros/grid_map_ros.hpp>

#define DEFAULT_MAP_NAME	"voronoi_map"

namespace voronoi_graph {
  
  

class VoronoiPathGenerator
{
  public:
    VoronoiPathGenerator ();
    void computeDistanceField ( const cv::Mat& _map, cv::Mat& _distField ) ;
    void computeVoronoiMap ( const cv::Mat& _distField, cv::Mat& _voronoiMap );
};

}

#endif
