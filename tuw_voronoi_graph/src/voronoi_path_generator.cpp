#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tuw_voronoi_map/voronoi_path_generator.h>
#include <tuw_voronoi_map/thinning.h>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <queue>
#include <string>

using namespace cv;

namespace voronoi_map
{

    VoronoiPathGenerator::VoronoiPathGenerator()
    {

    }

    void VoronoiPathGenerator::prepareMap(const Mat& _map, Mat& _des, int erodeSize)
    {
        static Mat srcMap;
        _map.convertTo(srcMap, CV_8UC1);

        for(int i = 0; i < srcMap.cols * srcMap.rows; i++)
        {
            if((signed char)_map.data[i] < 0)  srcMap.data[i] = 100;
        }

        _des = srcMap;

        cv::bitwise_not(srcMap, srcMap);
        cv::threshold(srcMap, _des, 10, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        
        if(erodeSize <= 0){
            erodeSize = 1;
        }
        if(erodeSize > 0){
            cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                        cv::Size( 2*erodeSize + 1, 2*erodeSize+1 ),
                                        cv::Point( erodeSize, erodeSize ) );
    
            cv::erode(_des, _des, element);
        }
    }


    void VoronoiPathGenerator::computeDistanceField(const cv::Mat& _map, cv::Mat& _distField)
    {
        cv::distanceTransform(_map, _distField, cv::DIST_L2, 3);
    }

    void VoronoiPathGenerator::computeVoronoiMap(const cv::Mat& _distField, cv::Mat& _voronoiMap)
    {
        Mat srcMap = _distField;
        srcMap.convertTo(_voronoiMap, CV_8UC1, 0.0);

        voronoi_map::greyscale_thinning(srcMap, _voronoiMap);
        cv::threshold(_voronoiMap, _voronoiMap, 1, 255, cv::THRESH_BINARY);
        voronoi_map::sceletonize(_voronoiMap, _voronoiMap);
    }
}
