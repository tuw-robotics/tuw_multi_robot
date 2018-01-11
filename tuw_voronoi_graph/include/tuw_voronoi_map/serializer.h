
#ifndef _TUW_VORONOI_SERIALIZER
#define _TUW_VORONOI_SERIALIZER

#include <nav_msgs/OccupancyGrid.h>
#include <memory>
#include <opencv/cv.h>
#include <grid_map_ros/grid_map_ros.hpp>


#include <fstream>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

#define DEFAULT_MAP_NAME	"voronoi_map"

namespace voronoi_map {
 
class VoronoiMapInfo
{
public:
  std::string mapName;
  std::string distanceMapName;
  std::string voronoiMapName;
  float resolution;
  int cols;
  int rows;
  float origin_cols;
  float origin_rows;
private:
  friend class boost::serialization::access;
  template<class archive> void serialize(archive & ar, const unsigned int version)
  {
    using boost::serialization::make_nvp;
      ar & boost::serialization::make_nvp("mapName",mapName);
      ar & boost::serialization::make_nvp("distanceMapName",distanceMapName);
      ar & boost::serialization::make_nvp("voronoiMapName",voronoiMapName);
      ar & boost::serialization::make_nvp("resolution",resolution);
      ar & boost::serialization::make_nvp("cols",cols);
      ar & boost::serialization::make_nvp("rows",rows);
      ar & boost::serialization::make_nvp("origin_cols",origin_cols);
      ar & boost::serialization::make_nvp("origin_rows",origin_rows);
  }
};
  
class VoronoiMapSerializer
{
public:
    VoronoiMapSerializer(int cols, int rows)
    {
      cols_ = cols;
      rows_ = rows;
      distfield.reset(new float[cols_*rows_]);
    }
    std::unique_ptr<float[]> distfield;
private:
  int cols_;
  int rows_;
  friend class boost::serialization::access;
  template<class archive> void serialize(archive & ar, const unsigned int version)
  {
	ar & boost::serialization::make_array<float>(distfield.get(), cols_*rows_);
  }
}; 
  
class VoronoiSerializer
{
public:
    VoronoiSerializer();
    VoronoiMapInfo getMapInfo();
    void saveMap(std::string mapInfoName, std::string mapPath);
    void loadMap(std::string mapInfoName, std::string mapPath);
protected:
    grid_map::GridMap map_;
    VoronoiMapInfo mapInfo_;
    
};

}

#endif
