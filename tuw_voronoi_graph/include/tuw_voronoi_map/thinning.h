#ifndef _THINNING
#define _THINNING

#include <opencv2/core/core.hpp>
#include <queue>

namespace voronoi_map
{
  
  /**
 * @brief Perform one thinning iteration.(Normally you wouldn't call this function directly from your code)
 * @param im    Binary image with range = [0,1]
 * @param iter  0=even, 1=odd
 */
void sceletonize_iteration(cv::Mat& img, int iter);
/**
 * @brief Function for thinning the given binary image	(Paper Zhang-Suen Thinning)
 * @param src  The source image, binary with range = [0,255]
 * @param dst  The destination image
 */
void sceletonize(const cv::Mat& src, cv::Mat& dst);
 
/**
 * @brief Function for finding the ridge of a distance graph
 * @param src	the source image containing the distance field (Mat float)
 * @param dst 	the destination image containing 0 for non voronoi graph pixels (else voronoigraph) (Mat uint8_t)
 */
void greyscale_thinning(const cv::Mat& src, cv::Mat& dst);


class Index
{
public: 
  Index(int x,int y, float pot)
  {
    i = x;
    j = y;
    potential = pot;
  }
  Index offset(int x, int y)
  {
    return Index(i+x,j+y, potential);
  }
  int i;
  int j;
  float potential;
};


/**
 * @brief function for finding the maximum Neighbour ignoring the last detected pixels
 * @param i the row of the pixel
 * @param j the column of the pixel
 * @param src	the source image containing the distance field (Mat float)
 * @param dst 	the destination image containing 0 for non voronoi graph pixels (else voronoigraph) (Mat uint8_t)
 */
Index getMaximumNeighbour(int i, int j, const cv::Mat& src, cv::Mat& dst);

}

#endif
