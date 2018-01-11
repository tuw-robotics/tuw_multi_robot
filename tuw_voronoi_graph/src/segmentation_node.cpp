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

#include <voronoi_segmentation/segmentation_node.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <boost/functional/hash.hpp>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "segmentation_node");     /// initializes the ros node with default name
    ros::NodeHandle n;

    ros::Rate r(0.3);

    Segmentation_Node node(n);

    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}


Segmentation_Node::Segmentation_Node(ros::NodeHandle &_n) :
    Segmentation(),
    n_(_n),
    n_param_("~")
{



    map_topic_ = "voronoi_map";
    n_param_.param("map_topic", map_topic_, map_topic_);

    segment_topic_ = "segments";
    n_param_.param("segment_topic", segment_topic_, segment_topic_);

    path_length_ = 0.9;   //meter
    n_param_.param("segment_length", path_length_, path_length_);

    subVoronoiMap_ = _n.subscribe(map_topic_, 1, &Segmentation_Node::mapCallback, this);

    pubSegments_ = _n.advertise<tuw_multi_robot_msgs::VoronoiGraph>(segment_topic_, 1);
    debug_pub_ = _n.advertise<nav_msgs::OccupancyGrid>("potential", 1);
}

void Segmentation_Node::mapCallback(const grid_map_msgs::GridMap& msg)
{
    grid_map::GridMap voronoiMap;
    grid_map::GridMapRosConverter::fromMessage(msg, voronoiMap);

    size_t hash = getHash(voronoiMap);

    if(hash != current_hash_)
    {
        current_hash_ = hash;

        geometry_msgs::PoseArray poses;
        poses.header.frame_id = "map";
        poses.header.seq = 0;
        poses.header.stamp = ros::Time::now();

        voronoi_map_ = std::make_shared<grid_map::GridMap>(voronoiMap);
        potential.reset(new float[voronoi_map_->getSize()[0] * voronoi_map_->getSize()[1]]);

        float pixel_path_length = path_length_ / voronoi_map_->getResolution();

        std::shared_ptr<std::vector<std::shared_ptr<Segment>>> segments = calcSegments(voronoi_map_, potential.get(), pixel_path_length);

        publishPotential(potential.get(), voronoi_map_->getSize()[0], voronoi_map_->getSize()[1], voronoi_map_->getResolution(), voronoi_map_->getPosition()[0], voronoi_map_->getPosition()[1]);

        tuw_multi_robot_msgs::VoronoiGraph graph;
        graph.header.frame_id = "map";
        graph.header.seq = 0;
        graph.header.stamp = ros::Time::now();

        graph.resolution = voronoi_map_->getResolution();

        grid_map::Position p_origin;
        grid_map::Index id(0);
        voronoi_map_->getPosition(id, p_origin);

        graph.origin.position.x = voronoi_map_->getSize()[0] * voronoi_map_->getResolution() - p_origin[0];
        graph.origin.position.y = voronoi_map_->getSize()[1] * voronoi_map_->getResolution() - p_origin[1];


        for(auto it = segments->begin(); it != segments->end(); ++it)
        {
            tuw_multi_robot_msgs::Vertex seg;
            seg.header.frame_id = "map";
            seg.header.seq = 0;
            seg.header.stamp = ros::Time::now();

            seg.id = (*it)->GetId();
            seg.length = (*it)->GetLength();
            seg.minPathSpace = (*it)->GetMinPathSpace();

            std::shared_ptr< std::vector< std::pair< int, int > > > path = (*it)->GetPath();

            for(int i = 0; i < path->size(); i++)
            {
                geometry_msgs::Point pos;
                pos.x = (*path)[i].first;
                pos.y = (*path)[i].second;
                pos.z = 0;

                seg.path.push_back(pos);
            }

            std::vector< std::shared_ptr< Segment > > predecessors = (*it)->GetPredecessors();

            for(int i = 0; i < predecessors.size(); i++)
            {
                seg.predecessor.push_back(predecessors[i]->GetId());
            }

            std::vector< std::shared_ptr< Segment > > successors = (*it)->GetSuccessors();

            for(int i = 0; i < successors.size(); i++)
            {
                seg.successors.push_back(successors[i]->GetId());
            }

            graph.segments.push_back(seg);
        }

        current_graph_ = graph;
		ROS_INFO("Map Hash %lu", hash);
    }

    current_graph_.header.stamp = ros::Time::now();
    pubSegments_.publish(current_graph_);
}


void Segmentation_Node::publishPotential(float* potential, int nx, int ny, double resolution, int cx, int cy)
{
    nav_msgs::OccupancyGrid grid;
    // Publish Whole Grid
    grid.header.frame_id = "map";
    grid.header.stamp = ros::Time::now();
    grid.info.resolution = resolution;

    grid.info.width = nx;
    grid.info.height = ny;

    double wx, wy;
    //costmap_->mapToWorld(0, 0, wx, wy);
    grid.info.origin.position.x = -nx * resolution / 2 + cx;
    grid.info.origin.position.y = -ny * resolution / 2 + cy;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    grid.data.resize(nx * ny);

    /*for (unsigned int i = 0; i < grid.data.size(); i++)
    {

        if (potential[i] == POT_HIGH)
        {
            potential[i] = -1;
        }
    }*/

    float max = 0.0;

    for(unsigned int i = 0; i < grid.data.size(); i++)
    {

        if(potential[i] > max)
        {
            max = potential[i];
        }
    }

    for(unsigned int i = 0; i < grid.data.size(); i++)
    {
        if(potential[i] != -1)
            grid.data[i] = 27.0 + potential[i] * 100.0 / max;
        else
            grid.data[i] = 0;
    }

    debug_pub_.publish(grid);
}

size_t Segmentation_Node::getHash(const grid_map::GridMap &_map)
{
    auto& map = _map.get("map");
    std::size_t seed = 0;

    for(grid_map::GridMapIterator iterator(_map); !iterator.isPastEnd(); ++iterator)
    {
        boost::hash_combine(seed, (float)map(iterator.getLinearIndex()));
    }

    return seed;
}
