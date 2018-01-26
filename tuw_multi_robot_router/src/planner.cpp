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

#include <tuw_global_planner/planner.h>
#include <tuw_global_planner/path_coordinator_timed.h>
#include <algorithm>
#include <limits>
#include <tuw_global_planner/velocity_calculator.h>
#include <chrono>

#include <time.h>

#include <ros/ros.h> //DEBUG


Planner::Planner() : Planner(0)
{
}


Planner::Planner(int _nr_robots) :
    paths_(_nr_robots),
    goals_(_nr_robots),
    robot_poses_(_nr_robots),
    startSegments_(_nr_robots),
    goalSegments_(_nr_robots),
    voronoiGoals_(_nr_robots),
    voronoiStart_(_nr_robots),
    radius_(_nr_robots)
{
    std::shared_ptr< Heuristic > heuristic =  std::make_shared< Heuristic >();

    pCalc_ = std::make_shared<PotentialCalculator>();
    path_querry_ = std::make_shared<Path_Coordinator_Timed>();
    resolution_ = std::make_shared<BacktrackingAvoidResolution> (path_querry_, pCalc_, 1); //TODO time overlap !!!                //TODO SELECT Whished coordinator dynamicly
    expander_ = std::make_unique<SegmentExpander> (heuristic, pCalc_, path_querry_, resolution_);
    traceback_ = std::make_unique<Traceback>();
    pointExpander_ = std::make_unique<PointExpander>();
    path_generator_Point_ = std::make_unique<PathGenerator<Potential_Point>> (path_querry_);
    path_generator_Segment_ = std::make_unique<PathGenerator<PathSegment>> (path_querry_);
    priorityScheduler_ = std::make_unique<PriorityScheduler> (_nr_robots);
    speedScheduler_ = std::make_unique<SpeedScheduler>(_nr_robots);
    velocityCalc_ = std::make_unique<VelocityCalculator>(_nr_robots);
}

void Planner::resize(int _nr_robots)
{
    goals_.resize(_nr_robots);
    robot_poses_.resize(_nr_robots);

    paths_.resize(_nr_robots);

    startSegments_.resize(_nr_robots);
    goalSegments_.resize(_nr_robots);

    voronoiGoals_.resize(_nr_robots);
    voronoiStart_.resize(_nr_robots);

    realGoals_.resize(_nr_robots);
    realStart_.resize(_nr_robots);

    radius_.resize(_nr_robots);

    priorityScheduler_->reset(_nr_robots);
    speedScheduler_->reset(_nr_robots);
    velocityCalc_->Init(_nr_robots);
}


bool Planner::calculateStartPoints(const std::vector<float> _radius, const grid_map::GridMap& _map, const std::vector<std::shared_ptr<Segment>>& _graph)
{
    int size_x = _map.getSize() [0];
    int size_y = _map.getSize() [1];

    potential_.reset(new float[size_x * size_y]);

    for(int i = 0; i < goals_.size(); i++)
    {
        grid_map::Index mapIdx;
        grid_map::Position p(robot_poses_[i].x, robot_poses_[i].y);
        _map.getIndex(p, mapIdx);
        Point start = { (float)(size_x - mapIdx[0]), (float)(size_y - mapIdx[1]) };
        Eigen::Vector2d s(start.x, start.y);


        grid_map::Position p2(goals_[i].x, goals_[i].y);
        _map.getIndex(p2, mapIdx);
        Point end = { (float)(size_x - mapIdx[0]), (float)(size_y - mapIdx[1]) };
        Eigen::Vector2d g(end.x, end.y);

        radius_[i] = ((float) _radius[i]) / (float) _map.getResolution();

        realGoals_[i] = end;
        realStart_[i] = start;


        int segId = getSegment(_graph, s);

        if(segId == -1)
            return false;

        ROS_INFO("Start %i", segId);
        startSegments_[i] = _graph[segId];
        voronoiStart_[i] = Point((_graph[segId]->getStart().x + _graph[segId]->getEnd().x) / 2, (_graph[segId]->getStart().y + _graph[segId]->getEnd().y) / 2);


        segId = getSegment(_graph, g);

        if(segId == -1)
            return false;

        ROS_INFO("Goal %i", segId);
        goalSegments_[i] = _graph[segId];
        voronoiGoals_[i] = Point((_graph[segId]->getStart().x + _graph[segId]->getEnd().x) / 2, (_graph[segId]->getStart().y + _graph[segId]->getEnd().y) / 2);
        bool found = false;


        if(!resolveSegment(_graph, startSegments_[i], start, radius_[i], startSegments_[i]))
        {
            ROS_INFO("WTF");
            return false;
        }

        if(!resolveSegment(_graph, goalSegments_[i], end, radius_[i], goalSegments_[i]))
        {
            ROS_INFO("WTF2");
            return false;
        }

    }

    return true;
}

//TODO Allow move to segment

int Planner::getSegment(const std::vector<std::shared_ptr<Segment>> &_graph, const Eigen::Vector2d &_odom)
{
    float minDist = FLT_MAX;
    int segment = -1;

    for(int i = 0; i < _graph.size(); i++)
    {
        ROS_INFO("seg %f %f, s %f %f, g %f %f, w %f", _odom[0], _odom[1], _graph[i]->getStart().x, _graph[i]->getStart().y, _graph[i]->getEnd().x, _graph[i]->getEnd().y, _graph[i]->getPathSpace());
        float d = distanceToSegment(_graph[i], _odom);

        if(d < minDist && d <= _graph[i]->getPathSpace())
        {
            segment = i;
            minDist = d;
        }
    }

    return segment;
}

float Planner::distanceToSegment(std::shared_ptr<Segment> _s, Eigen::Vector2d _p)
{
    Eigen::Vector2d goal(_s->getEnd().x, _s->getEnd().y);
    Eigen::Vector2d start(_s->getStart().x, _s->getStart().y);

    Eigen::Vector2d n =  goal - start;
    Eigen::Vector2d pa = start - _p;

    float c = n.dot(pa);

    // Closest point is a
    if(c > 0.0f)
        return pa.dot(pa);

    Eigen::Vector2d bp = _p - goal;

    // Closest point is b
    if(n.dot(bp) > 0.0f)
        return bp.dot(bp);

    // Closest point is between a and b
    Eigen::Vector2d e = pa - n * (c / n.dot(n));


    return std::sqrt(e.dot(e));
}

bool Planner::resolveSegment(const std::vector< std::shared_ptr< Segment > >& _graph, const std::shared_ptr<Segment>& _seg, const Point& _originPoint, float _radius,  std::shared_ptr< Segment >& _foundSeg)
{

    if(_seg->isEdgeSegment() && _seg->getPathSpace() < _radius)
    {
        Neighbours nbs = _seg->getPredecessors();

        if(nbs.size() == 0)
        {
            nbs = _seg->getSuccessors();
        }

        if(nbs.size() == 0)
        {
            return true;
        }



        float dist = std::numeric_limits<float>::max();

        for(auto nb = nbs.cbegin(); nb != nbs.cend(); nb++)
        {
            float ds_x = _originPoint.x - (*nb)->getStart().x;
            float ds_y = _originPoint.y - (*nb)->getStart().y;
            float de_x = _originPoint.x - (*nb)->getEnd().x;
            float de_y = _originPoint.y - (*nb)->getEnd().y;
            float d = (std::sqrt(ds_x * ds_x + ds_y * ds_y) + std::sqrt(de_x * de_x + de_y * de_y)) / 2;

            if(d < dist)
            {
                _foundSeg = (*nb);
                dist = d;
            }
        }

        return true;

    }
    else if(_radius <= _seg->getPathSpace())
    {
        return true;
    }

    return false;
}


bool Planner::getPaths(const std::vector< std::shared_ptr< Segment > >& _graph, int& _actualRobot, const std::vector<int>& _priorityList, const std::vector<float>& _speedList)
{

    std::vector<std::vector<std::shared_ptr<Segment>>> paths(_priorityList.size());

    for(int i = 0; i < goals_.size(); i++)
    {
        _actualRobot = _priorityList[i];
        paths_[_actualRobot].clear();
        path_querry_->setActive(_actualRobot);
        pCalc_->SetMultiplier(_speedList[i]);


        ROS_INFO("Calculating Path %i...", _actualRobot);

        if(!expander_->calculatePotentials(startSegments_[_actualRobot], goalSegments_[_actualRobot], _graph, radius_[_actualRobot]))
        {
            return false;
        }


        ROS_INFO("Tracing back");

        std::vector< std::shared_ptr< Segment> > path;

        if(!traceback_->getPath(startSegments_[_actualRobot], goalSegments_[_actualRobot], path))
        {
            return false;
        }

        std::reverse(path.begin(), path.end());


        ROS_INFO("Checking Path");

        paths[_actualRobot] = (path);

        //Add path to p_querry
        if(!path_querry_->addPath(path , radius_[_actualRobot]))
        {
            ROS_INFO("Checking Path failed");
            return false;
        }

    }


    std::vector<std::vector<Potential_Point>> p;

    if(useGoalOnSegment_)
        p = path_generator_Point_->generatePath(paths, voronoiStart_, voronoiGoals_);
    else
        p =  path_generator_Point_->generatePath(paths, realStart_, realGoals_);

    paths_.clear();

    for(auto & genPath : p)
    {
        paths_.push_back(genPath);

        if(genPath.size() == 0)
        {
            return false;
        }
    }

    std::vector<std::vector<PathSegment>> pSegs;

    if(useGoalOnSegment_)
        pSegs = path_generator_Segment_->generatePath(paths, voronoiStart_, voronoiGoals_);
    else
        pSegs = path_generator_Segment_->generatePath(paths, realStart_, realGoals_);

    segPaths_.clear();

    for(auto & genSeg : pSegs)
    {
        segPaths_.push_back(genSeg);

        if(genSeg.size() == 0)
        {
            return false;
        }
    }

    longestPatLength_ = 0;

    //Test Overall Path length
    float length = 0;

    for(std::vector<std::shared_ptr<Segment>> &path : paths)
    {
        int lenP = 0;

        for(std::shared_ptr<Segment> &seg : path)
        {
            length += seg->getLength();
            lenP += seg->getLength();
        }

        longestPatLength_ = std::max<int>(longestPatLength_, lenP);

    }

    //ROS_INFO("Overall Path Length: %f\n", length);
    overallPathLength_ = length;



    return true;
}



bool Planner::makePlan(const std::vector< Point >& _goals, const std::vector<float>& _radius, const grid_map::GridMap& _map, const std::vector<std::shared_ptr<Segment>>& _graph)
{
    speedScheduleAttemps_ = 0;
    priorityScheduleAttemps_ = 0;
    auto t1 = std::chrono::high_resolution_clock::now();
    std::clock_t startcputime = std::clock();

    if(_goals.size() != goals_.size())
    {
        ROS_INFO("Wrong nr of goals %lu %lu", _goals.size(), goals_.size());
        return false;
    }

    ROS_INFO("=========================================================");
    pointExpander_->initialize(_map);
    goals_ = _goals;                //TODO check if ok

    if(!calculateStartPoints(_radius, _map, _graph))
    {
        return false;
    }


    priorityScheduler_->reset(_goals.size());
    int lastPlannedRobot = -1;
    std::vector<int> priorityList = priorityScheduler_->getActualSchedule();
    std::vector<int> collisions;

    do
    {
        ROS_INFO("=========================================================");
        ROS_INFO("New prority schedule");
        speedScheduler_->reset(_goals.size());
        std::vector<float> speedList = speedScheduler_->getActualSpeeds();

        do
        {
            ROS_INFO("=========================================================");
            ROS_INFO("New speed schedule");
            std::vector<std::shared_ptr<Segment>> graph = _graph;
            path_querry_->reset(graph, goals_.size());

            if(!path_querry_->setStartSegments(startSegments_, radius_))
            {
                return false;
            }

            if(!path_querry_->setGoalSegments(goalSegments_, radius_))
            {
                return false;
            }


            if(getPaths(graph, lastPlannedRobot, priorityList, speedList))
            {
                auto t2 = std::chrono::high_resolution_clock::now();
                duration_ = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();

                return true;
            }

            speedScheduleAttemps_++;

            collisions = path_querry_->getNrOfRobotCollisions(lastPlannedRobot);
        }
        while(speedScheduler_->rescheduleSpeeds(lastPlannedRobot, collisions, speedList));

        priorityScheduleAttemps_++;
    }
    while(priorityScheduler_->reschedulePriorities(lastPlannedRobot, collisions, priorityList));

    auto t2 = std::chrono::high_resolution_clock::now();
    duration_ = (std::clock() - startcputime) / (double)CLOCKS_PER_SEC * 1000;

    return false;
}


void Planner::updateRobotPose(int _robot_id, const Point& _pose)
{
    robot_poses_[_robot_id] = _pose;
}

const std::vector< Potential_Point >& Planner::getPath(int _robot_id)
{
    return paths_[_robot_id];
}

bool Planner::gotPlan()
{
    return gotPlan_;
}

const std::vector< PathSegment >& Planner::getPathSeg(int _robot_id)
{
    return segPaths_[_robot_id];
}


const std::vector<float>& Planner::getVelocityProfile(int _robot_id)
{
    return velocityProfile_[_robot_id];
}

int Planner::getDuration_ms()
{
    return duration_;
}

int Planner::getLongestPathLength()
{
    return longestPatLength_;
}

int Planner::getOverallPathLength()
{
    return overallPathLength_;
}

int Planner::getPriorityScheduleAttemps()
{
    return priorityScheduleAttemps_;
}

int Planner::getSpeedScheduleAttemps()
{
    return speedScheduleAttemps_;
}
