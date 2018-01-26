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
#define POT_HIGH 1.0e10

#include <tuw_global_planner/planner_node.h>
#include <tuw_global_planner/segment.h>
#include <tuw_multi_robot_msgs/SegmentPath.h>
#include <chrono>

int main(int argc, char** argv)
{

    ros::init(argc, argv, "tuw_nav_costmap_node");     /// initializes the ros node with default name
    ros::NodeHandle n;

    Planner_Node node(n);
    ros::Rate r(5);

    while(ros::ok())
    {
        r.sleep();
        ros::spinOnce();
    }

    return 0;
}


Planner_Node::Planner_Node(ros::NodeHandle& _n) :
    Planner(),
    n_(_n),
    n_param_("~"),
    robot_names_(std::vector<std::string> ( {"robot0"})),
             robot_radius_(std::vector<float> ( {1}))
{
    id_ = 0;
    n_param_.param("robot_names", robot_names_, std::vector<std::string>());

    resize(robot_names_.size());

    subOdom_.resize(robot_names_.size());
    pubPaths_.resize(robot_names_.size());
    pubSegPaths_.resize(robot_names_.size());
    pubVelocityProfile_.resize(robot_names_.size());

	useGoalOnSegment_ = false;
	n_param_.param("use_segment_as_goal", useGoalOnSegment_, useGoalOnSegment_);
	
	allowEndpointOffSegment_ = true;
	n_param_.param("allow_endpoint_off_segments", allowEndpointOffSegment_, allowEndpointOffSegment_);
	
    planner_status_topic_ = "planner_status";
    n_param_.param("planner_status_topic", planner_status_topic_, planner_status_topic_);

    odom_topic_ = "odom";
    n_param_.param("odom_topic", odom_topic_, odom_topic_);

    path_topic_ = "path";
    n_param_.param("path_topic", path_topic_, path_topic_);

    goal_topic_ = "goals";
    n_param_.param("goal_topic", goal_topic_, goal_topic_);

    map_topic_ = "voronoi_map";
    n_param_.param("map_topic", map_topic_, map_topic_);

    voronoi_topic_ = "segments";
    n_param_.param("graph_topic", voronoi_topic_, voronoi_topic_);

    segpath_topic_ = "seg_path";
    n_param_.param("seg_path", segpath_topic_, segpath_topic_);

    velocity_topic_ = "vel_profile";
    n_param_.param("velocity_topic_", velocity_topic_, velocity_topic_);

    n_param_.param("robot_radius", robot_radius_, std::vector<float>());


    for(size_t i = 0; i < subOdom_.size(); i++)
    {
        subOdom_[i] = _n.subscribe<nav_msgs::Odometry> (robot_names_[i] + "/" + odom_topic_, 1, boost::bind(&Planner_Node::odomCallback, this, _1, i));
        pubPaths_[i] = _n.advertise<nav_msgs::Path> (robot_names_[i] + "/" + path_topic_, 1);
        pubSegPaths_[i] = _n.advertise<tuw_multi_robot_msgs::SegmentPath> (robot_names_[i] + "/" + segpath_topic_, 1);
        pubVelocityProfile_[i] = _n.advertise<std_msgs::Float32MultiArray>(robot_names_[i] + "/" + velocity_topic_, 1);
    }

    subGoalSet_ = _n.subscribe(goal_topic_, 1, &Planner_Node::goalsCallback, this);
    subVoronoiMap_ = _n.subscribe(map_topic_, 1, &Planner_Node::mapCallback, this);
    subVoronoiGraph_ = _n.subscribe(voronoi_topic_, 1, &Planner_Node::graphCallback, this);


    debug_pub_ = _n.advertise<nav_msgs::OccupancyGrid> ("potential", 1);

    pubPlannerStatus_ = _n.advertise<tuw_multi_robot_msgs::PlannerStatus> (planner_status_topic_, 1);
}

void Planner_Node::mapCallback(const grid_map_msgs::GridMap& msg)
{
    grid_map::GridMapRosConverter::fromMessage(msg, voronoi_map_);
    got_map_ = true;
}


void Planner_Node::odomCallback(const ros::MessageEvent<nav_msgs::Odometry const>& _event, int _robot_nr)
{
    const nav_msgs::Odometry_< std::allocator< void > >::ConstPtr& nav_msg = _event.getMessage();

    Point p;
    p[0] = nav_msg->pose.pose.position.x;
    p[1] = nav_msg->pose.pose.position.y;

    updateRobotPose(_robot_nr, p);
}


void Planner_Node::graphCallback(const tuw_multi_robot_msgs::VoronoiGraph& msg)
{
    std::vector<std::shared_ptr<Segment>> graph;

    for(int i = 0; i < msg.segments.size(); i++)
    {
        std::vector<Point> points;

        for(int j = 0; j < msg.segments[i].path.size(); j++)
        {
            points.emplace_back(msg.segments[i].path[j].x, msg.segments[i].path[j].y);
        }

        graph.push_back(std::make_shared<Segment> (msg.segments[i].id, (float) msg.segments[i].minPathSpace, points));
    }

    std::sort(graph.begin(), graph.end(), sortSegments);


    for(auto & path_segment : msg.segments)
    {
        int index = path_segment.id;

        for(int i = 0; i < path_segment.successors.size(); i++)
        {
            graph[index]->addSuccessor(graph[path_segment.successors[i]]);
        }

        for(int i = 0; i < path_segment.predecessor.size(); i++)
        {
            graph[index]->addPredecessor(graph[path_segment.predecessor[i]]);
        }
    }

    graph_ = graph;

    got_graph_ = true;
}


void Planner_Node::goalsCallback(const tuw_multi_robot_msgs::PoseIdArray& _goals)
{
    //Goals have to be orderd
    std::vector<Point> goals;

    for(auto it = _goals.poses.begin(); it != _goals.poses.end(); it++)
    {
        Point p;
        p[0] = (*it).position.x;
        p[1] = (*it).position.y;
        goals.push_back(p);
    }

    if(got_map_ && got_graph_)
    {
        auto t1 = std::chrono::high_resolution_clock::now();

        if(makePlan(goals, robot_radius_, voronoi_map_, graph_))
        {
            int nx = voronoi_map_.getSize() [0];
            int ny = voronoi_map_.getSize() [1];


            double res = voronoi_map_.getResolution();
            int cx = voronoi_map_.getPosition() [0];
            int cy = voronoi_map_.getPosition() [1];
            //publishPotential(potential_.get(), nx, ny, res, cx, cy);  //Debug

            Publish();
            ROS_INFO("Publishing Plan");
        }
        else
        {
            ROS_INFO("No Plan found");
            int nx = voronoi_map_.getSize() [0];
            int ny = voronoi_map_.getSize() [1];

            PublishEmpty();
        }

        auto t2 = std::chrono::high_resolution_clock::now();
        int duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
        ROS_INFO("OverallTime %i ms", duration);

        id_++;
    }
}

void Planner_Node::PublishEmpty()
{
    for(int i = 0; i < robot_names_.size(); i++)
    {
        nav_msgs::Path ros_path;
        ros_path.header.seq = 0;
        ros_path.header.stamp = ros::Time::now();
        ros_path.header.frame_id = "map";

        pubPaths_[i].publish(ros_path);
    }

    for(int i = 0; i < robot_names_.size(); i++)
    {
        tuw_multi_robot_msgs::SegmentPath ros_path;
        ros_path.header.seq = 0;
        ros_path.header.stamp = ros::Time::now();
        ros_path.header.frame_id = "map";

        pubPaths_[i].publish(ros_path);
    }

    tuw_multi_robot_msgs::PlannerStatus ps;
    ps.id = id_;
    ps.success = 0;
    ps.duration = getDuration_ms();

    pubPlannerStatus_.publish(ps);

}


void Planner_Node::Publish()
{
	
    if(gotPlan())
    {
		//ORIGIN ??
		grid_map::Index origS(0,0);
		grid_map::Index id(0);
		grid_map::Position orig;
		grid_map::Position p_origin;
		voronoi_map_.getPosition(origS, orig);
		voronoi_map_.getPosition(id, p_origin);
		
		p_origin[0] = voronoi_map_.getSize()[0] * voronoi_map_.getResolution() - p_origin[0] - orig[0];
		p_origin[1] = voronoi_map_.getSize()[1] * voronoi_map_.getResolution() - p_origin[1] - orig[1];
		
        for(int i = 0; i < robot_names_.size(); i++)
        {
            nav_msgs::Path ros_path;
            ros_path.header.seq = 0;
            ros_path.header.stamp = ros::Time::now();
            ros_path.header.frame_id = "map";
            const std::vector< Potential_Point >& path = getPath(i);

            for(auto it = path.cbegin(); it != path.cend(); it++)
            {
                geometry_msgs::PoseStamped p;
                p.header.seq = 0;
                p.header.stamp = ros::Time::now();
                p.header.frame_id = "map";
                grid_map::Index idx((*it).point[0], (*it).point[1]);
                grid_map::Position pos;
                voronoi_map_.getPosition(idx, pos);
                p.pose.position.x = -pos[0] - p_origin[0];
                p.pose.position.y = -pos[1] - p_origin[1];
				
				p.pose.orientation.w = 1;
                ros_path.poses.push_back(p);

            }

            pubPaths_[i].publish(ros_path);
        }

        for(int i = 0; i < robot_names_.size(); i++)
        {
            tuw_multi_robot_msgs::SegmentPath ros_path;
            ros_path.header.seq = 0;
            ros_path.header.stamp = ros::Time::now();
            ros_path.header.frame_id = "map";
            const std::vector< PathSegment >& path = getPathSeg(i);

            for(auto it = path.cbegin(); it != path.cend(); it++)
            {
                tuw_multi_robot_msgs::PathSegment s;

                grid_map::Index idxS((*it).start[0], (*it).start[1]);
                grid_map::Position posS;
                voronoi_map_.getPosition(idxS, posS);

				
				
                s.start.x = -posS[0] - p_origin[0];
                s.start.y = -posS[1] - p_origin[1];

                grid_map::Index idxE((*it).end[0], (*it).end[1]);
                grid_map::Position posE;
                voronoi_map_.getPosition(idxE, posE);
                s.end.x = -posE[0] - p_origin[0];
                s.end.y = -posE[1] - p_origin[1];

                s.segId = (*it).segId;
				s.width = graph_[(*it).segId]->getPathSpace() * voronoi_map_.getResolution();	//TODO		(SEG ID WRONG)

                for(int j = 0; j < (*it).preconditions.size(); j++) //auto precond = (*it).preconditions.cbegin(); precond != (*it).preconditions.cend(); precond++)
                {
                    tuw_multi_robot_msgs::PathPrecondition pc;
                    pc.robotId = ((*it).preconditions[j]).robot;
                    pc.stepCondition = ((*it).preconditions[j]).stepCondition;
                    s.preconditions.push_back(pc);
                }

                ros_path.poses.push_back(s);
            }

            pubSegPaths_[i].publish(ros_path);
        }


        tuw_multi_robot_msgs::PlannerStatus ps;
        ps.id = id_;
        ps.success = 1;
        ps.overallPathLength = getOverallPathLength();
        ps.longestPathLength = getLongestPathLength();
        ps.prioritySchedulingAttemps = getPriorityScheduleAttemps();
        ps.speedSchedulingAttemps = getSpeedScheduleAttemps();
        ps.duration = getDuration_ms();

        pubPlannerStatus_.publish(ps);
    }
}


void Planner_Node::publishPotential(float* potential, int nx, int ny, double resolution, int cx, int cy)
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

    for(unsigned int i = 0; i < grid.data.size(); i++)
    {

        if(potential[i] == POT_HIGH)
        {
            potential[i] = -1;
        }
    }

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
        {
            grid.data[i] = 27.0 + potential[i] * 100.0 / max;
        }
        else
        {
            grid.data[i] = 0;
        }
    }

    debug_pub_.publish(grid);
}


