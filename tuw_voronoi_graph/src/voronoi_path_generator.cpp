#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tuw_voronoi_map/voronoi_path_generator.h>
#include <tuw_voronoi_map/thinning.h>
#include <memory>
#include <opencv/cv.hpp>
#include <queue>
#include <string>

using namespace cv;

namespace voronoi_graph
{

    VoronoiPathGenerator::VoronoiPathGenerator()
    {

    }

    void VoronoiPathGenerator::computeDistanceField(const cv::Mat& _map, cv::Mat& _distField)
    {
        static Mat srcMap;
        _map.convertTo(srcMap, CV_8UC1);
        
        for(int i = 0; i < srcMap.cols * srcMap.rows; i++)
        {
              if((signed char)_map.data[i] < 0)
                  srcMap.data[i] = 100;
        }
        
        cv::bitwise_not(srcMap, srcMap);
        cv::threshold(srcMap, srcMap, 40, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);

        cv::distanceTransform(srcMap, _distField, CV_DIST_L2, 3);
    }
    
    void VoronoiPathGenerator::computeVoronoiMap(const cv::Mat& _distField, cv::Mat& _voronoiMap)
    {
        Mat srcMap = _distField;
        srcMap.convertTo(_voronoiMap, CV_8UC1, 0.0);
        
        voronoi_map::greyscale_thinning(srcMap, _voronoiMap);
        cv::threshold(_voronoiMap, _voronoiMap, 1, 255, CV_THRESH_BINARY);
        voronoi_map::sceletonize(_voronoiMap, _voronoiMap);
    }



}
