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
#include <opencv2/core/core.hpp>

namespace tuw_graph
{

    class Segment_Expander
    {
        public:
            Segment_Expander();
            
            /** 
             * @brief initializes the expander by setting the voronoi path map and distancefield 
             * @param _map the map 
             * @param _distField the distance_field generated from the map (e.g.: opencv distance_transform)
             * @param _voronoiPath the generated voronoi path out of the distance field (e.g.: ridge following; zhang suen thinning...)
             */
            void Initialize(cv::Mat &_map, cv::Mat &_distField, cv::Mat &_voronoiPath);
            /**
             * @brief looks for crossings and saves all segment endpoints of it
             * @param _potential voronoi map potential
             * @return a list of crossings which contains all endpoints of the crossing 
             */
            std::vector<std::vector<Eigen::Vector2d>> calcEndpoints(float *_potential);
            
            /**
             * @brief returns a list of segments, which represent the found search graph 
             * @param _endPoints a list of all found endpoints (list of crossings which contain a set of endpoints)
             * @param _potential potential used to expand the pats
             * @param _min_length the minimum length a segment has to have (max = _min_length * 2 - epsilon)
             * @param _optimizePixelsCrossing if crossings have less distance than _optimizePixelsCrossing to each other they are merged
             * @param _optimizePixelsEndSegments if a end segment has less length than this var it is removed
             * @return a list of segments representing the graph
             */
            std::vector< Segment > getGraph(const std::vector<std::vector<Eigen::Vector2d>> &_endPoints, float *_potential, const float _min_length, float _optimizePixelsCrossing, const float _optimizePixelsEndSegments);

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
                    Index offsetDist(int dist_x, int dist_y, int nx, int ny) const
                    {
                        int x_val = (i % nx) + dist_x;
                        int y_val = (i / nx) + dist_y;

                        if(x_val < 0 || x_val > nx || y_val < 0 || y_val > ny)
                            return Index(-1, -1, -1, -1);

                        return Index(x_val, y_val, nx, 0, 0, 0);
                    }
                    int getX(int nx) const
                    {
                        return (i % nx);
                    }
                    int getY(int nx) const
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
        
            //Splits a given path into multiple segments with minimum length
            const std::vector<tuw_graph::Segment> splitPath(const std::vector<Eigen::Vector2d> &_path, const float _minimum_length);
 
            //Returns the nr of neighbors (removing double neighbours...)
            uint32_t nrOfNeighbours(uint32_t i) const;
            
            //searches all path endpoints at a crossing
            std::vector<Eigen::Vector2d> expandCrossing(const Index &i, float* _potential);
            
            //expands from a given crossing endpoint until another crossing is found
            Eigen::Vector2d expandSegment(Index start, float* _potential, const std::vector<std::vector<Eigen::Vector2d>> &_endpoints);  
            
            //Tries to find a path
            std::vector<Eigen::Vector2d> getPath(const Index &start, float* _potential, const std::vector<std::vector<Eigen::Vector2d>> &_endpoints);

            //Returns the minimum space a segment has
            float getMinSegmentWidth(const std::vector<Eigen::Vector2d> &_path);

            //Safely removes segments from the segment list taking care of neighbors
            void removeSegmentFromList(const uint32_t _id, std::vector<Segment> &_segments);
            
            //Adds an expansion candidate for the Dijkstra algorithm
            void addExpansionCandidate(const Index &current, const Index &next, float* potential);
            
            //Checks if the given point is a valid endpoint
            bool isEndpoint(Index &_current, const std::vector<std::vector<Eigen::Vector2d>> &_endpoints);
            
            //Removes a endponit 
            void removeEndpoint(const Index &_current, std::vector<std::vector<Eigen::Vector2d>> &_endpoints);
            bool checkSegmentPoint(const Index &_point);
            
            //Optimizes the Graph by removing to short segments at the end of it and merging crossings in a specific radius
            void optimizeSegments(std::vector<Segment> &_segments, float _maxPixelsCrossing, float _maxPixelsEndSeg);
            
            //Merges crossings together
            void optimizeSegmentsAroundPoint(std::vector<Segment> &_segments, const Eigen::Vector2d &pt, float maxPixels, int startIndex);
    };

}
#endif // VORONOI_EXPANDER_H
