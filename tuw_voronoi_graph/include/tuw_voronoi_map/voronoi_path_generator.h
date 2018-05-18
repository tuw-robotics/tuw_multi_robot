
#ifndef _GENERATOR
#define _GENERATOR

#include <nav_msgs/OccupancyGrid.h>
#include <memory>
#include <opencv/cv.h>

#define DEFAULT_MAP_NAME    "voronoi_map"

namespace voronoi_map
{
    class VoronoiPathGenerator
    {
        public:
            VoronoiPathGenerator();
            void prepareMap(const cv::Mat& _map, cv::Mat& _smoothedMap, int blurSize);
            void computeDistanceField(const cv::Mat& _map, cv::Mat& _distField) ;
            void computeVoronoiMap(const cv::Mat& _distField, cv::Mat& _voronoiMap);
    };

}

#endif
