#include <ros/ros.h>
#include <tuw_voronoi_map/serializer.h>
#include <memory>
#include <opencv/cv.hpp>
#include <queue> 
#include <string>


#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

using namespace cv;

namespace voronoi_map {
  
VoronoiSerializer::VoronoiSerializer()
{

}

/**
 * @brief loads the map in the given location
 * @param mapInfo the name of the mapInfo file
 **/
void VoronoiSerializer::loadMap(std::string mapInfoName, std::string mapPath)
{   
  std::ifstream ifs ( mapPath + mapInfoName );
  assert ( ifs.good() );
  boost::archive::xml_iarchive xml ( ifs );
  xml >> boost::serialization::make_nvp ( "MapInfo", mapInfo_ );
  
  
 // map_.setTimestamp(message.info.header.stamp.toNSec());
 // map_.setFrameId(message.info.header.frame_id);
  
  VoronoiMapSerializer mapMap(mapInfo_.cols,mapInfo_.rows);
  std::ifstream ifsMap(mapPath + mapInfo_.mapName);
  boost::archive::binary_iarchive iaMap(ifsMap);
  iaMap >> mapMap;
  
  
  VoronoiMapSerializer distanceMap(mapInfo_.cols,mapInfo_.rows);
  std::ifstream ifsDist(mapPath + mapInfo_.distanceMapName);
  boost::archive::binary_iarchive iaDist(ifsDist);
  iaDist >> distanceMap;
  
  
  VoronoiMapSerializer voronoiMap(mapInfo_.cols, mapInfo_.rows);
  std::ifstream ifsVoronoi(mapPath + mapInfo_.voronoiMapName);
  boost::archive::binary_iarchive iaVoronoi(ifsVoronoi);
  iaVoronoi >> voronoiMap;
  
  
  map_.setGeometry(grid_map::Length(mapInfo_.cols * mapInfo_.resolution, mapInfo_.rows * mapInfo_.resolution), mapInfo_.resolution, grid_map::Position(mapInfo_.origin_cols, mapInfo_.origin_rows));

  
    map_.add("map");
    auto& layerM = map_["map"];
    
    for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator) 
    {	
      layerM(iterator.getLinearIndex()) = mapMap.distfield[iterator.getLinearIndex()];
    }
  
    map_.add("distfield");
    auto& layerD = map_["distfield"];
    
    for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator) 
    {
      layerD(iterator.getLinearIndex()) = distanceMap.distfield[iterator.getLinearIndex()];
    }
    
    map_.add("voronoi");
    auto& layerV = map_["voronoi"];
    
    for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator) 
    {
      layerV(iterator.getLinearIndex()) = voronoiMap.distfield[iterator.getLinearIndex()];
    }
}

/**
 * @brief saves the map in the given location
 * @param mapInfo the name of the mapInfo file
 **/
void VoronoiSerializer::saveMap(std::string mapInfoName, std::string mapPath)
{
  mapInfo_.mapName = "map";
  mapInfo_.distanceMapName = "distfield";
  mapInfo_.voronoiMapName = "voronoi";
  mapInfo_.resolution=map_.getResolution();
  mapInfo_.cols=map_.getSize()(0);
  mapInfo_.rows=map_.getSize()(1);
  mapInfo_.origin_cols=map_.getPosition()(0);
  mapInfo_.origin_rows=map_.getPosition()(1); 
  
  std::ofstream ofs(mapPath + mapInfoName);
  assert(ofs.good());
  boost::archive::xml_oarchive oa(ofs);
  oa << boost::serialization::make_nvp ("MapInfo",mapInfo_);
  
  ROS_INFO("%s", (mapPath + mapInfoName).c_str());
  
  VoronoiMapSerializer mapMap(mapInfo_.cols,mapInfo_.rows);
  
  auto& layerM = map_[mapInfo_.mapName];
  for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator) 
  {
    //ROS_INFO("%f",layerM(iterator.getLinearIndex()));
    if(layerM(iterator.getLinearIndex()) != layerM(iterator.getLinearIndex()))
      mapMap.distfield[iterator.getLinearIndex()] = -1;
    else
      mapMap.distfield[iterator.getLinearIndex()] = layerM(iterator.getLinearIndex());
    //else
    //  mapMap.distfield[iterator.getLinearIndex()] = -1;
  }
  
  std::ofstream ofsMap(mapPath + mapInfo_.mapName);
  boost::archive::binary_oarchive oaMap(ofsMap);
  oaMap << mapMap;
  
  
  VoronoiMapSerializer distanceMap(mapInfo_.cols,mapInfo_.rows);
  
  auto& layerSrc = map_[mapInfo_.distanceMapName];
  for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator) 
  {
    distanceMap.distfield[iterator.getLinearIndex()] = layerSrc(iterator.getLinearIndex());
  }
  
  std::ofstream ofsDistance(mapPath + mapInfo_.distanceMapName);
  boost::archive::binary_oarchive oaDist(ofsDistance);
  oaDist << distanceMap;
  
  
  VoronoiMapSerializer voronoiMap(mapInfo_.cols,mapInfo_.rows);
  
  auto& layerV = map_[mapInfo_.voronoiMapName];
  for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator) 
  {
    voronoiMap.distfield[iterator.getLinearIndex()] = layerV(iterator.getLinearIndex());
  }
  
  std::ofstream ofsVoronoi(mapPath + mapInfo_.voronoiMapName);
  boost::archive::binary_oarchive oaVor(ofsVoronoi);
  oaVor << voronoiMap;
  
  
}

VoronoiMapInfo VoronoiSerializer::getMapInfo()
{
  return mapInfo_;
}


  
}
