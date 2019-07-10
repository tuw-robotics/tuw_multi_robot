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

#include <tuw_voronoi_graph/segment_to_graph_node.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <boost/functional/hash.hpp>
#include "yaml-cpp/yaml.h"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "segment to graph"); /// initializes the ros node with default name
    ros::NodeHandle n;

    ros::Rate r(0.3);

    tuw_graph::SegmentToGraphNode node(n);

    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();
        node.Publish();
    }

    return 0;
}

namespace tuw_graph
{
SegmentToGraphNode::SegmentToGraphNode(ros::NodeHandle &_n) : n_(_n),
                                                              n_param_("~")
{
    segment_topic_ = "segments";
    n_param_.param("segment_topic", segment_topic_, segment_topic_);

    segment_file_ = "segments.yaml";
    n_param_.param("segment_file", segment_file_, segment_file_);

    path_length_ = 0.9; //meter
    n_param_.param("segment_length", path_length_, path_length_);

    pubSegments_ = _n.advertise<tuw_multi_robot_msgs::Graph>(segment_topic_, 1);

    readSegments();
}

void SegmentToGraphNode::Publish()
{
    pubSegments_.publish(current_graph_);
}

void SegmentToGraphNode::readSegments()
{
    ROS_INFO("%s", segment_file_.c_str());
    YAML::Node waypoints_yaml = YAML::LoadFile(segment_file_);
    std::vector<double> start_x = (waypoints_yaml["start_x"].as<std::vector<double>>());
    std::vector<double> start_y = (waypoints_yaml["start_y"].as<std::vector<double>>());
    std::vector<double> end_x = (waypoints_yaml["end_x"].as<std::vector<double>>());
    std::vector<double> end_y = (waypoints_yaml["end_y"].as<std::vector<double>>());
    std::vector<double> path_space = (waypoints_yaml["space"].as<std::vector<double>>());
    double origin_x = (waypoints_yaml["origin_x"].as<double>());
    double origin_y = (waypoints_yaml["origin_y"].as<double>());
    double resolution = (waypoints_yaml["resolution"].as<double>());

    std::vector<PathSeg> graph;

    for (uint32_t i = 0; i < start_x.size(); i++)
    {
        Eigen::Vector2d start((start_x[i] + origin_x) / resolution, (start_y[i] + origin_y) / resolution);
        Eigen::Vector2d end((end_x[i] + origin_x) / resolution, (end_y[i] + origin_y) / resolution);
        graph.emplace_back(start, end, path_space[i]);
    }

    //TODO Precompute

    current_graph_.vertices.clear();
    current_graph_.header.frame_id = "map";
    current_graph_.header.seq = 0;
    current_graph_.header.stamp = ros::Time::now();
    current_graph_.origin.position.x = origin_x;
    current_graph_.origin.position.y = origin_y;

    for (uint32_t i = 0; i < graph.size(); i++)
    {
        tuw_multi_robot_msgs::Vertex v;
        v.id = i;

        v.weight = (graph[i].start - graph[i].end).norm();
        v.width = graph[i].width / resolution;

        geometry_msgs::Point start;
        start.x = graph[i].start[0];
        start.y = graph[i].start[1];
        start.z = 0;
        geometry_msgs::Point end;
        end.x = graph[i].end[0];
        end.y = graph[i].end[1];
        end.z = 0;

        v.path.push_back(start);
        v.path.push_back(end);

        std::vector<int> pred = findNeighbors(graph, graph[i].start, i);

        for (int pr : pred)
        {
            v.predecessors.push_back(pr);
        }

        std::vector<int> succ = findNeighbors(graph, graph[i].end, i);

        for (int sr : succ)
        {
            v.successors.push_back(sr);
        }

        current_graph_.vertices.push_back(v);
    }

    ROS_INFO("Segments parsed");
}

std::vector<int32_t> SegmentToGraphNode::findNeighbors(std::vector<PathSeg> &_graph, Eigen::Vector2d _point, uint32_t _segment)
{
    std::vector<int32_t> n;

    for (uint32_t i = 0; i < _graph.size(); i++)
    {
        if (i != _segment)
        {
            if ((_graph[i].start - _point).norm() < 0.1 || (_graph[i].end - _point).norm() < 0.1)
            {
                n.push_back(i);
            }
        }
    }

    return n;
}

} // namespace tuw_graph
