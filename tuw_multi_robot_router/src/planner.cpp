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
#include <chrono>
#include <unordered_set>

#include <time.h>

#include <ros/ros.h>

namespace multi_robot_router
{

    Planner::Planner() : Planner(0)
    {
    }


    Planner::Planner(const uint32_t _nr_robots) :
        robot_poses_(_nr_robots),
        goals_(_nr_robots),
        pose_received_(_nr_robots, false)
    {
        robot_nr_ = _nr_robots;
        pointExpander_ = std::make_unique<PointExpander>();
        std::vector<uint32_t> robotRadius(_nr_robots, 0);
        multiRobotRouter_ = std::make_unique<MultiRobotRouter>(_nr_robots, robotRadius);

    }

    void Planner::setCollisionResolutionType(const SegmentExpander::CollisionResolverType _cr)
    {
        multiRobotRouter_->setCollisionResolver(_cr);
    }


    void Planner::resize(const uint32_t _nr_robots)
    {
        robot_nr_ = _nr_robots;
        robot_poses_.resize(_nr_robots);
        pose_received_.resize(_nr_robots, false);
        goals_.resize(_nr_robots);

        startSegments_.resize(_nr_robots);
        goalSegments_.resize(_nr_robots);

        voronoiGoals_.resize(_nr_robots);
        voronoiStart_.resize(_nr_robots);

        realGoals_.resize(_nr_robots);
        realStart_.resize(_nr_robots);

        diameter_.resize(_nr_robots);

    }


    bool Planner::calculateStartPoints(const std::vector<float> &_radius, const cv::Mat &_map, const float &resolution, const Eigen::Vector2d &origin, const std::vector<Segment> &_graph)
    {
        for(uint32_t i = 0; i < goals_.size(); i++)
        {
            if(!pose_received_[i])
            {
                ROS_INFO("No Start Pose for robot %i received", i);
                return false;
            }

            //Find Start and Goal Poses On the map
            realStart_[i] = { (robot_poses_[i][0] - origin[0]) / resolution, (robot_poses_[i][1] - origin[1]) / resolution };
            realGoals_[i] = { (goals_[i][0] - origin[0]) / resolution, (goals_[i][1] - origin[1]) / resolution };

            diameter_[i] = 2 * ((float) _radius[i]) / resolution;

            int32_t segIdStart = -1;
            int32_t segIdGoal = -1;

            //check if start and goal pose have enough clearance to obstacles
            uint32_t size_x = _map.cols;
            uint32_t size_y = _map.rows;

            if(pointExpander_->getDistanceToObstacle(realStart_[i]) < diameter_[i] / 2)
            {
                ROS_INFO("Start of robot %i is to close to an obstacle", i);
                return false;
            }

            if(pointExpander_->getDistanceToObstacle(realGoals_[i]) < diameter_[i] / 2)
            {
                ROS_INFO("Goal of robot %i is to close to an obstacle", i);
                return false;
            }


            //No algorithm checks free space to obstacle because it is allready checked
            if(graphMode_ == graphType::voronoi)
            {
                //Find Start and Goal (with distance to Segment (more performance but robot has to be inside a segment)
                segIdStart = getSegment(_graph, realStart_[i]);
                segIdGoal = getSegment(_graph, realGoals_[i]);

                if(segIdStart != -1 && segIdGoal != -1)
                {
                    voronoiStart_[i] = (_graph[segIdStart].getStart() + _graph[segIdStart].getEnd()) / 2;
                    voronoiGoals_[i] = (_graph[segIdGoal].getStart() + _graph[segIdGoal].getEnd()) / 2;
                }
            }
            else// if(graphMode_ == graphType::random)
            {
                //Save all segment Points into map to find them using point Expander
                std::map<uint32_t, Eigen::Vector2d> points;

                for(const Segment & seg : _graph)
                {
                    std::pair<uint32_t, Eigen::Vector2d> p(seg.getSegmentId(), (seg.getStart() + seg.getEnd()) / 2);
                    points.insert(p);
                }

                //find Segment using Dijkstra (less performance but find segment for sure)
                potential_.reset(new float[size_x * size_y]);

                pointExpander_->findGoalOnMap(realStart_[i], size_x * size_y, potential_.get(), points, 0, voronoiStart_[i], segIdStart, diameter_[i] / 2); //It is allready checked if there is enough free space
                potential_.reset(new float[size_x * size_y]);
                pointExpander_->findGoalOnMap(realGoals_[i], size_x * size_y, potential_.get(), points, 0, voronoiGoals_[i], segIdGoal, diameter_[i] / 2); //It is allready checked if there is enough free space
            }


            if(segIdStart == -1)
            {
                ROS_INFO("Start of robot %i was not found", i);
                return false;
            }

            if(segIdGoal == -1)
            {
                ROS_INFO("Goal of robot %i was not found", i);
                return false;
            }


            startSegments_[i] = segIdStart;
            goalSegments_[i] = segIdGoal;

            //Optimize found segments for errors
            if(!resolveSegment(_graph, startSegments_[i], realStart_[i], diameter_[i], startSegments_[i]))
            {
                ROS_INFO("Start of robot %i is not valid", i);
                return false;
            }

            if(!resolveSegment(_graph, goalSegments_[i], realGoals_[i], diameter_[i], goalSegments_[i]))
            {
                ROS_INFO("Goal of robot %i is not valid", i);
                return false;
            }

            //ROS_INFO("DEBUG Robot %i: StartSeg[%i]; GoalSeg[%i]", i, startSegments_[i], goalSegments_[i]);
        }

        return true;
    }

    int32_t Planner::getSegment(const std::vector<Segment> &_graph, const Eigen::Vector2d &_odom)
    {
        float minDist = FLT_MAX;
        int32_t segment = -1;

        //Select the segment which contains the robot center
        for(uint32_t i = 0; i < _graph.size(); i++)
        {
            float d = distanceToSegment(_graph[i], _odom);

            if(d < minDist && d <= _graph[i].width())
            {
                segment = i;
                minDist = d;
            }
        }

        return segment;
    }

    float Planner::distanceToSegment(const Segment &_s, const Eigen::Vector2d &_p)
    {
        Eigen::Vector2d n =  _s.getEnd() - _s.getStart();
        Eigen::Vector2d pa = _s.getStart() - _p;

        float c = n.dot(pa);

        // Closest point is a
        if(c > 0.0f)
            return std::sqrt(pa.dot(pa));

        Eigen::Vector2d bp = _p - _s.getEnd();

        // Closest point is b
        if(n.dot(bp) > 0.0f)
            return std::sqrt(bp.dot(bp));

        // Closest point is between a and b
        Eigen::Vector2d e = pa - n * (c / n.dot(n));

        return std::sqrt(e.dot(e));
    }

    bool Planner::resolveSegment(const std::vector< Segment > &_graph, const uint32_t &_segId, const Eigen::Vector2d &_originPoint, const float &_diameter,  uint32_t &_foundSeg)
    {
        const Segment seg = _graph[_segId];

        if((seg.getPredecessors().size() == 0 || seg.getSuccessors().size() == 0) && seg.width() < _diameter)
        {
            //If we are on a leave Segment we are allowed to move the robot one segment in the graph if its radius is to big.
            //Because it can happen, that a leave segment has a triangular shape and thus wrong width value.
            std::vector<uint32_t> neighbours;

            if(seg.getPredecessors().size() != 0)
                neighbours = seg.getPredecessors();
            else
                neighbours = seg.getSuccessors();

            float dist = std::numeric_limits<float>::max();

            //Find the closest neighbour
            for(const uint32_t & neighbour : neighbours)
            {
                float ds_x = _originPoint[0] - _graph[neighbour].getStart()[0];
                float ds_y = _originPoint[1] - _graph[neighbour].getStart()[1];
                float de_x = _originPoint[0] - _graph[neighbour].getEnd()[0];
                float de_y = _originPoint[1] - _graph[neighbour].getEnd()[1];
                float d = (std::sqrt(ds_x * ds_x + ds_y * ds_y) + std::sqrt(de_x * de_x + de_y * de_y)) / 2;

                if(d < dist && _diameter <= _graph[neighbour].width())
                {
                    _foundSeg = neighbour;
                    dist = d;
                }
            }

            //Return only true if a valid Segment is found, where the radius of the robot is smaller
            //than the segment width
            if(_diameter <= _graph[_foundSeg].width())
                return true;

        }
        else if(_diameter <= seg.width())
        {
            //if the radius is smaller than the segment width a valid segment is found
            return true;
        }

        //No valid wait segment is found
        return false;
    }

    bool Planner::makePlan(const std::vector< Eigen::Vector2d > &_goals, const std::vector<float> &_radius, const cv::Mat &_map, const float &_resolution, const Eigen::Vector2d &_origin, const std::vector<Segment> &_graph)
    {
        auto t1 = std::chrono::high_resolution_clock::now();
        std::clock_t startcputime = std::clock();

        if(_goals.size() != robot_nr_)
        {
            ROS_INFO("Multi Robot Router: Wrong nr of goals %lu %lu", _goals.size(), goals_.size());
            auto t2 = std::chrono::high_resolution_clock::now();
            duration_ = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
            return false;
        }

        ROS_INFO("=========================================================");
        pointExpander_->initialize(_map);
        goals_ = _goals;

        if(!calculateStartPoints(_radius, _map, _resolution, _origin, _graph))
        {
            ROS_INFO("Multi Robot Router: Failed to find Endpoints");
            auto t2 = std::chrono::high_resolution_clock::now();
            duration_ = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
            return false;
        }

        multiRobotRouter_->setRobotNr(robot_nr_);

        std::vector<uint32_t> diameter;

        for(int i = 0; i < _radius.size(); i++)
        {
            diameter.push_back(2 * ((float) _radius[i]) / _resolution);
        }

        multiRobotRouter_->setRobotRadius(diameter);
        multiRobotRouter_->setPriorityRescheduling(priorityRescheduling_);
        multiRobotRouter_->setSpeedRescheduling(speedRescheduling_);
        routingTable_.clear();

        if(!multiRobotRouter_->getRoutingTable(_graph, startSegments_, goalSegments_, routingTable_, routerTimeLimit_s_))
        {
            ROS_INFO("Failed to find Routing Table");
            auto t2 = std::chrono::high_resolution_clock::now();
            duration_ = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
            return false;
        }

        if(segmentOptimizations_)
            optimizePaths(_graph);

        postprocessRoutingTable();



        //DEBUG STATS
        longestPatLength_ = 0;
        overallPathLength_ = 0;

        for(std::vector<Checkpoint> &path : routingTable_)
        {
            float lengthPath = 0;

            for(Checkpoint & seg : path)
            {
                Eigen::Vector2d vec = (seg.end - seg.start);
                float lengthVertex = std::sqrt(vec[0] * vec[0] + vec[1] * vec[1]);
                overallPathLength_ += lengthVertex;
                lengthPath += lengthVertex;
            }

            longestPatLength_ = std::max<int>(longestPatLength_, lengthPath);
        }

        longestPatLength_ *= _resolution;
        overallPathLength_ *= _resolution;

        auto t2 = std::chrono::high_resolution_clock::now();
        duration_ = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
        //DEBUG STATS

        return true;
    }

    void Planner::optimizePaths(const std::vector<Segment> &_graph)
    {
        if(routingTable_.size() > 1)
            return;
                
        routingTable_[0].erase(routingTable_[0].begin(), routingTable_[0].begin() + 1);
        routingTable_[0].erase(routingTable_[0].end() - 1, routingTable_[0].end());
    }


    void Planner::postprocessRoutingTable()
    {
        if(goalMode_ == goalMode::use_voronoi_goal)
        {
            for(int i = 0; i < routingTable_.size(); i++)
            {
                routingTable_[i].front().start = voronoiStart_[i];
                routingTable_[i].back().end = voronoiGoals_[i];
            }
        }
        else if(goalMode_ == goalMode::use_map_goal)
        {
            for(int i = 0; i < routingTable_.size(); i++)
            {
                routingTable_[i].front().start = realStart_[i];
                routingTable_[i].back().end = realGoals_[i];
            }
        }
    }



    const std::vector< Checkpoint > &Planner::getRoute(const uint32_t _robot)
    {
        return routingTable_[_robot];
    }


    void Planner::updateRobotPose(const uint32_t _robot_id, const Eigen::Vector2d &_pose)
    {
        robot_poses_[_robot_id] = _pose;
        pose_received_[_robot_id] = true;
    }


    uint32_t Planner::getDuration_ms()
    {
        return duration_;
    }

    float Planner::getLongestPathLength()
    {
        return longestPatLength_;
    }

    float Planner::getOverallPathLength()
    {
        return overallPathLength_;
    }

    uint32_t Planner::getPriorityScheduleAttemps()
    {
        return multiRobotRouter_->getPriorityScheduleAttempts();
    }

    uint32_t Planner::getSpeedScheduleAttemps()
    {
        return multiRobotRouter_->getSpeedScheduleAttempts();
    }
}
