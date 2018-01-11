#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tuw_voronoi_map/generator.h>
#include <tuw_voronoi_map/thinning.h>
#include <memory>
#include <opencv/cv.hpp>
#include <queue> 
#include <string>

using namespace cv;

namespace voronoi_map {
  
VoronoiGenerator::VoronoiGenerator()
{
}


void VoronoiGenerator::computeDistanceField(grid_map::GridMap& _gridMap, const std::string& _layerSrc, const std::string& _layerDes)
{
    static Mat srcMap;
    auto& layerSrc = _gridMap[_layerSrc];
    srcMap = Mat(_gridMap.getSize()(1), _gridMap.getSize()(0), CV_8UC1);
   	
    for (grid_map::GridMapIterator iterator(_gridMap); !iterator.isPastEnd(); ++iterator) 
    {
      const grid_map::Index mapIndex = iterator.getUnwrappedIndex();
      if(layerSrc(iterator.getLinearIndex()) == 0)			//if free space
	srcMap.at<uint8_t>(mapIndex[1],mapIndex[0]) = 100;
      else 
	srcMap.at<uint8_t>(mapIndex[1],mapIndex[0]) = 0;
    }
    
    Mat destMap_f;  
    cv::distanceTransform ( srcMap, destMap_f, CV_DIST_L2, 3 );
    
    _gridMap.add(_layerDes);
    auto& layer = _gridMap[_layerDes];
    
    for (grid_map::GridMapIterator iterator(_gridMap); !iterator.isPastEnd(); ++iterator) 
    {
      const grid_map::Index mapIndex = iterator.getUnwrappedIndex();		
      layer(iterator.getLinearIndex()) = destMap_f.at<float_t>(mapIndex[1], mapIndex[0]);
    }
}
void VoronoiGenerator::computeVoronoiMap(grid_map::GridMap& _gridMap, const std::string& _layerSrc, const std::string& _layerDes)
{
    static Mat srcMap;
    srcMap = Mat(_gridMap.getSize()(1), _gridMap.getSize()(0), CV_32FC1);
    auto& layerSrc = _gridMap[_layerSrc];
   	
    for (grid_map::GridMapIterator iterator(_gridMap); !iterator.isPastEnd(); ++iterator) 
    {
      const grid_map::Index mapIndex = iterator.getUnwrappedIndex(); 	
      srcMap.at<float_t>(mapIndex[1],mapIndex[0]) = layerSrc(iterator.getLinearIndex());
    }
    
    Mat destMap_f;  
    srcMap.convertTo(destMap_f, CV_8UC1, 0.0);
    
    voronoi_map::greyscale_thinning(srcMap,destMap_f);
    cv::threshold(destMap_f, destMap_f, 1, 255, CV_THRESH_BINARY);
    voronoi_map::sceletonize(destMap_f, destMap_f);
        
    _gridMap.add(_layerDes);
    auto& layer = _gridMap[_layerDes];
    
    for (grid_map::GridMapIterator iterator(_gridMap); !iterator.isPastEnd(); ++iterator) 
    {
      const grid_map::Index mapIndex = iterator.getUnwrappedIndex();	
      layer(iterator.getLinearIndex()) = destMap_f.at<uint8_t>(mapIndex[1],mapIndex[0]);
    }
}
  

  
}