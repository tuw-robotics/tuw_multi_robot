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

#ifndef VORONOI_GRAPH_GENERATOR_H
#define VORONOI_GRAPH_GENERATOR_H

#include <ros/ros.h>
#include <tuw_voronoi_graph/segment.h>
#include <tuw_voronoi_graph/segment_expander.h>

namespace tuw_graph
{
    class VoronoiGraphGenerator
    {
        public:     
            VoronoiGraphGenerator();
            /** 
             * @brief calculates the search graph 
             * @param _map the map 
             * @param _distField the distance_field generated from the map (e.g.: opencv distance_transform)
             * @param _voronoiPath the generated voronoi path out of the distance field (e.g.: ridge following; zhang suen thinning...)
             * @param potential the potential used for expanding (debuggin purpose)
             * @param _path_length the minimum path length a segment has to have
             * @param _optimizePixelsCrossing if crossings have less distance than _optimizePixelsCrossing to each other they are merged
             * @param _optimizePixelsEndSegments if a end segment has less length than this var it is removed
             */
            std::vector<Segment> calcSegments(cv::Mat &_map, cv::Mat &_distField, cv::Mat &_voronoiPath, float * potential, float _path_length, float _optimizeCrossingPixels, float _optimizeEndSegmentsPixel);

    };
}
#endif // PLANNER_NODE_H
