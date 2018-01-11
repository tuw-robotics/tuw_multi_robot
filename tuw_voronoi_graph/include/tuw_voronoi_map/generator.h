
#ifndef _GENERATOR
#define _GENERATOR

#include <nav_msgs/OccupancyGrid.h>
#include <memory>
#include <opencv/cv.h>
#include <grid_map_ros/grid_map_ros.hpp>

#define DEFAULT_MAP_NAME	"voronoi_map"

namespace voronoi_map {
  
  

class VoronoiGenerator
{
  public:
    /**
     * @brief constructor
     */
    VoronoiGenerator();
    /**
     * @brief computes the distance field of the map using opencv
     * @param _gridMap the grid map containing the src and dst _layerDes
     * @param _layerSrc the source layer containing the map
     * @param _layerDes the layer the distfield is written to
     */
    void computeDistanceField ( grid_map::GridMap& _gridMap, const std::string& _layerSrc, const std::string& _layerDes ) ;
    /**
     * @brief computes the voronoi map using Greyscale thinning (not exactly but basically (A.Nedzved, S. Uchida, S. Ablameyko)) and thinning (Zang, Suen) thinning to skeletonize the found voronoi graph 
     * @param _gridMap the grid map containing the src and dst _layerDes
     * @param _layerSrc the source layer containing the distfield
     * @param _layerDes the layer the graph is written to
     */
    void computeVoronoiMap(grid_map::GridMap& _gridMap, const std::string& _layerSrc, const std::string& _layerDes);

  private:
    nav_msgs::OccupancyGrid distanceMap_;
    cv::Mat voronoi_map_;
    cv::Mat gradient_map_;
    int data_length_;
    int width_;
    int height_;
    float mapResolution_;
    ros::Time mapLoadTime_;
    geometry_msgs::Pose mapOrigin_;
    int id_ = 0;
    std::string map_name_ = DEFAULT_MAP_NAME;  
    int mapMaxGradientValue_ = 100;
    int mapVoroniMapVlaue_ = -1;
};

}

#endif
