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
#include <grid_map_ros/grid_map_ros.hpp>
#include <voronoi_segmentation/segment.h>

class Segment_Expander
{
private: 		std::unique_ptr<float[]> distance_field_;
private: 		std::unique_ptr<int8_t[]> voronoi_graph_;
private: 		std::unique_ptr<int8_t[]> global_map_;
private:		bool findEdgeSegments_ = true;
private:		int nx_, ny_, ns_;
private:
template <class T, class S, class C>
        void clearpq(std::priority_queue<T, S, C>& q){
            q=std::priority_queue<T, S, C>();
        }
        class Index {
            public:
                Index(int index, float c, float d, float p) {
                    i = index;
                    weight = c;
		    dist = d;
		    potential = p;
                }
                Index(int x_val, int y_val, int nx, float c, float d, float p){
		  i = x_val + y_val*nx;
		  weight = c;
		  dist = d;
		  potential = p;
		}
		Index offsetDist(int dist_x, int dist_y, int nx, int ny)
		{
		  int x_val = (i % nx) + dist_x;
		  int y_val = (i / nx) + dist_y;
		  if(x_val < 0 || x_val > nx || y_val < 0 || y_val > ny)
		    return Index(-1,-1,-1,-1);
		  return Index(x_val,y_val,nx,0,0,0);
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
        struct greater1 {
                bool operator()(const Index& a, const Index& b) const {
                    return a.weight > b.weight;
                }
        };
        std::priority_queue<Index, std::vector<Index>, greater1> queue_;
		



public: 		Segment_Expander();
	
public: 		void Initialize(std::shared_ptr<grid_map::GridMap> _map);
public: 		std::shared_ptr<std::vector<std::vector<std::pair<int,int>>>> calcEndpoints(float *_potential);
public: 		std::shared_ptr<std::vector<std::pair<std::pair<float, float>,std::pair<float, float>>>> getSegments(std::shared_ptr<std::vector<std::vector<std::pair<int,int>>>> _endPoints, float *_potential);	//TODO Deprecated
public: 		std::pair<float, float> getSegmentNeighbour(std::pair<float,float> _p, std::shared_ptr< std::vector< std::vector< std::pair< int, int > > > > _endpoints);											//TODO Deprecated

public:			std::vector< std::shared_ptr<Segment> > getGraph(std::shared_ptr<std::vector<std::vector<std::pair<int,int>>>> _endPoints, float *_potential);
public:			std::vector< std::shared_ptr<Segment> > getGraph(std::shared_ptr<std::vector<std::vector<std::pair<int,int>>>> _endPoints, float *_potential, float _max_length);

private:		void getMaps(float *distance_field, int8_t *voronoi_graph, int8_t *global_map, grid_map::GridMap *voronoi_map);
private:		int nrOfNeighbours(int i);
private:		std::vector<std::pair<int,int>> expandCrossing(Index i, float* _potential);
private:		std::pair<int,int> expandSegment(Index start, float* _potential, std::shared_ptr<std::vector<std::vector<std::pair<int,int>>>> _endpoints); 														//TODO Deprecated
private:		std::vector<std::pair<int,int>> getPath(Index start, float* _potential, std::shared_ptr<std::vector<std::vector<std::pair<int,int>>>> _endpoints, float &min_d);

private:		float getMinD(std::vector<std::pair<int,int>> _path);


private:		void addExpansionCandidate(Index current, Index next, float* potential);
private:		bool isEndpoint(Index _current, std::shared_ptr<std::vector<std::vector<std::pair<int,int>>>> _endpoints);
private:		void removeEndpoint(Index _current, std::shared_ptr<std::vector<std::vector<std::pair<int,int>>>> _endpoints);
private:		bool checkSegmentPoint(Index _point);
};

#endif // VORONOI_EXPANDER_H
