/*
 * Copyright (c) 2017, <copyright holder> <email>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY <copyright holder> <email> ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <copyright holder> <email> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef SEGMENT_EXPANDER_H
#define SEGMENT_EXPANDER_H

#include <algorithm>
#include <memory>
#include <queue>
#include <vector>
#include <tuw_voronoi_graph/segment.h>
#include <tuw_voronoi_graph/crossing.h>
#include <opencv/cv.hpp>

namespace tuw_graph
{

    class Segment_Expander
    {

        public:
            Segment_Expander();

            void Initialize(cv::Mat &_map, cv::Mat &_distField, cv::Mat &_voronoiPath);
            std::vector<std::vector<Eigen::Vector2d>> calcEndpoints(float *_potential);
            std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> getSegments(const std::vector<std::vector<Eigen::Vector2d>> &_endPoints, float *_potential);
            Eigen::Vector2d getSegmentNeighbour(const Eigen::Vector2d &_p, const std::vector< std::vector< Eigen::Vector2d > > &_endpoints);                                          //TODO Deprecated
            std::vector< Segment > getGraph(const std::vector<std::vector<Eigen::Vector2d>> &_endPoints, float *_potential, float _max_length, float _optimizePixelsCrossing, float _optimizePixelsEndSegments);

        private:
            std::unique_ptr<float[]> distance_field_;
            std::unique_ptr<int8_t[]> voronoi_graph_;
            std::unique_ptr<int8_t[]> global_map_;
            bool findEdgeSegments_ = true;
            int nx_, ny_, ns_;

            template <class T, class S, class C>
            void clearpq(std::priority_queue<T, S, C>& q)
            {
                q = std::priority_queue<T, S, C>();
            }
            class Index
            {
                public:
                    Index(int index, float c, float d, float p)
                    {
                        i = index;
                        weight = c;
                        dist = d;
                        potential = p;
                    }
                    Index(int x_val, int y_val, int nx, float c, float d, float p)
                    {
                        i = x_val + y_val * nx;
                        weight = c;
                        dist = d;
                        potential = p;
                    }
                    Index offsetDist(int dist_x, int dist_y, int nx, int ny)
                    {
                        int x_val = (i % nx) + dist_x;
                        int y_val = (i / nx) + dist_y;

                        if(x_val < 0 || x_val > nx || y_val < 0 || y_val > ny)
                            return Index(-1, -1, -1, -1);

                        return Index(x_val, y_val, nx, 0, 0, 0);
                    }
                    int getX(int nx)
                    {
                        return (i % nx);
                    }
                    int getY(int nx)
                    {
                        return (i / nx);
                    }
                    int i;
                    float weight;
                    float dist;
                    float cost;
                    float potential;
            };
            struct greater1
            {
                bool operator()(const Index& a, const Index& b) const
                {
                    return a.weight > b.weight;
                }
            };
            std::priority_queue<Index, std::vector<Index>, greater1> queue_;

            int nrOfNeighbours(int i);
            std::vector<Eigen::Vector2d> expandCrossing(const Index &i, float* _potential);
            Eigen::Vector2d expandSegment(Index start, float* _potential, const std::vector<std::vector<Eigen::Vector2d>> &_endpoints);                                                        //TODO Deprecated
            std::vector<Eigen::Vector2d> getPath(const Index &start, float* _potential, const std::vector<std::vector<Eigen::Vector2d>> &_endpoints, float &min_d);

            float getMinD(const std::vector<Eigen::Vector2d> &_path);

            void removeSegmentFromList(const uint32_t _id, std::vector<Segment> &_segments);
            void addExpansionCandidate(const Index &current, const Index &next, float* potential);
            bool isEndpoint(Index &_current, const std::vector<std::vector<Eigen::Vector2d>> &_endpoints);
            void removeEndpoint(Index _current, std::vector<std::vector<Eigen::Vector2d>> &_endpoints);
            bool checkSegmentPoint(const Index &_point);
            void optimizeSegments(std::vector<Segment> &_segments, float _maxPixelsCrossing, float _maxPixelsEndSeg);
            void optimizeSegmentsAroundPoint(std::vector<Segment> &_segments, const Eigen::Vector2d &pt, float maxPixels, int startIndex);
    };

}
#endif // VORONOI_EXPANDER_H
