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

#include <voronoi_segmentation/segmentation.h>
#include <voronoi_segmentation/segment_expander.h>


Segmentation::Segmentation() 
{
  
}

std::shared_ptr< std::vector< std::shared_ptr< Segment > > > Segmentation::calcSegments(std::shared_ptr< grid_map::GridMap > _map, float* potential, float _path_length)
{
  Segment_Expander exp;
  exp.Initialize(_map);
  std::shared_ptr<std::vector<std::vector<std::pair<int,int>>>> points = exp.calcEndpoints(potential);
  std::vector<std::pair<std::pair<float,float>,std::pair<float,float>>> segments;
  
  
  
  int nx = _map->getSize()[0];
  int ny = _map->getSize()[1];
  
  exp.Initialize(_map);
  std::fill(potential, potential + nx*ny, -1);
  auto segs = exp.getGraph(points,potential, _path_length);//TODO TEST
  
  
  for (int i = 0; i < segs.size(); i++)
  {
	ROS_INFO("Segment %i", segs[i]->GetId());
	ROS_INFO("\t (%i/%i) (%i/%i)", segs[i]->getStart().first, segs[i]->getStart().second, segs[i]->getEnd().first, segs[i]->getEnd().second);
	ROS_INFO("\t l: %i l: %f", segs[i]->GetLength(), segs[i]->GetMinPathSpace());
	
	std::vector<std::shared_ptr<Segment>> predecessors = segs[i]->GetPredecessors();
	std::vector<std::shared_ptr<Segment>> successors = segs[i]->GetSuccessors();
	
	
	for(int j = 0; j < predecessors.size(); j++)
	{
	  ROS_INFO("\t Predecessor %i", predecessors[j]->GetId());
	}
	
	for(int j = 0; j < successors.size(); j++)
	{
	  ROS_INFO("\t Successor %i", successors[j]->GetId());
	}
	ROS_INFO(" ");
  }
  
	ROS_INFO(" ");
	ROS_INFO(" ");
	ROS_INFO(" ");
  
  return std::make_shared<std::vector<std::shared_ptr<Segment>>>(segs);
}



std::shared_ptr<std::vector<std::pair<std::pair<float, float>,std::pair<float, float>>>> Segmentation::getSegments(std::shared_ptr< grid_map::GridMap > _map, float* potential)
{
  Segment_Expander exp;
  exp.Initialize(_map);
  std::shared_ptr<std::vector<std::vector<std::pair<int,int>>>> points = exp.calcEndpoints(potential);
  std::vector<std::pair<std::pair<float,float>,std::pair<float,float>>> segments;
  
  
  
  int nx = _map->getSize()[0];
  int ny = _map->getSize()[1];
  
  exp.Initialize(_map);
  std::fill(potential, potential + nx*ny, -1);
  auto a = exp.getGraph(points,potential);
  
  for(int i = 0; i < a.size(); i++)
  {
	std::pair<float, float> p1((a[i]->getStart().first - nx/2) * _map->getResolution(), (a[i]->getStart().second - ny/2) * _map->getResolution());
	std::pair<float, float> p2((a[i]->getEnd().first - nx/2) * _map->getResolution(), (a[i]->getEnd().second - ny/2) * _map->getResolution());
	
	std::pair<std::pair<float,float>,std::pair<float,float>> seg(p1,p2);
	
	segments.push_back(seg);
  }
  
  for (int i = 0; i < a.size(); i++)
  {
	ROS_INFO("Segment %i", a[i]->GetId());
	ROS_INFO("\t (%i/%i) (%i/%i)", a[i]->getStart().first, a[i]->getStart().second, a[i]->getEnd().first, a[i]->getEnd().second);
	ROS_INFO("\t l: %i l: %f", a[i]->GetLength(), a[i]->GetMinPathSpace());
	
	std::vector<std::shared_ptr<Segment>> predecessors = a[i]->GetPredecessors();
	std::vector<std::shared_ptr<Segment>> successors = a[i]->GetSuccessors();
	
	
	for(int j = 0; j < predecessors.size(); j++)
	{
	  ROS_INFO("\t Predecessor %i", predecessors[j]->GetId());
	}
	
	for(int j = 0; j < successors.size(); j++)
	{
	  ROS_INFO("\t Successor %i", successors[j]->GetId());
	}
	ROS_INFO(" ");
  }
  
	ROS_INFO(" ");
	ROS_INFO(" ");
	ROS_INFO(" ");
  
  return std::make_shared<std::vector<std::pair<std::pair<float,float>,std::pair<float,float>>>>(segments);
}
