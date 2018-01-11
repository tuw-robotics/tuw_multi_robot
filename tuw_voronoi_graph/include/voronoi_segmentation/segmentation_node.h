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

#ifndef SEGMENTATION_NODE_H
#define SEGMENTATION_NODE_H

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <memory>
#include <grid_map_ros/grid_map_ros.hpp>
#include <tuw_multi_robot_msgs/Vertex.h>
#include <tuw_multi_robot_msgs/VoronoiGraph.h>

#include <voronoi_segmentation/segmentation.h>

class Segmentation_Node : Segmentation
{
public:		Segmentation_Node(ros::NodeHandle &n);

public: 	void Publish();
//ROS:
public:		ros::NodeHandle 			n_;      ///< Node handler to the root node
public:		ros::NodeHandle 			n_param_;///< Node handler to the current node
public:		std::unique_ptr<ros::Rate> 		rate_;

// ROS Publishers
private:	ros::Publisher				 pubSegments_;
private:	ros::Subscriber              subVoronoiMap_; 


private:	std::shared_ptr<grid_map::GridMap>	voronoi_map_;
private: 	std::string				segment_topic_;
private:	std::string				map_topic_;
private:	float 					path_length_;
private: 	std::unique_ptr<float[]> potential;

private: 	tuw_multi_robot_msgs::VoronoiGraph current_graph_;
private: 	size_t current_hash_;


private:	void publishPotential(float* potential, int nx, int ny, double resolution, int cx, int cy);
private:	ros::Publisher debug_pub_;	//DEBUG
private: 	void mapCallback(const grid_map_msgs::GridMap& msg);

private:	size_t getHash(const grid_map::GridMap &_map);

};

#endif // PLANNER_NODE_H
