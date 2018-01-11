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

#ifndef SEGMENT_H
#define SEGMENT_H

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>


class Segment
{
public:		Segment(std::shared_ptr<std::vector<std::pair<int,int>>> _points, float _min_space);

public:		void AddPredecessor(std::shared_ptr<Segment> _predecessor);
public:		void AddSuccessor(std::shared_ptr<Segment> _successor);
public:		std::shared_ptr<std::vector<std::pair<int,int>>> GetPath();
public:		void SetPath(std::shared_ptr<std::vector<std::pair<int,int>>> _path);
public:		float GetMinPathSpace();
public:		void SetMinPathSpace(float _space);
public:		int GetLength();

public:		static void ResetId();

public:		std::pair<int,int> getStart();
public:		std::pair<int,int> getEnd();

public:		int GetId();
public:		std::vector<std::shared_ptr<Segment>> GetPredecessors();
public:		std::vector<std::shared_ptr<Segment>> GetSuccessors();

private:	std::pair<int,int> start_, end_;
private:	float min_space_;
private:	float length_;
private:	std::shared_ptr<std::vector<std::pair<int,int>>> wayPoints_;

private:	std::vector<std::shared_ptr<Segment>> successor_;
private:	std::vector<std::shared_ptr<Segment>> predecessor_;

private:	static int static_id_;
private:	int id_;
};

#endif // PLANNER_NODE_H
