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

#include <tuw_global_router/router.h>
#include <chrono>
#include <unordered_set>

#include <time.h>
#include <thread>
#include <ros/ros.h>

namespace multi_robot_router
{

Router::Router() : Router(0)
{
}

Router::Router(const uint32_t _nr_robots) : starts_(_nr_robots),
                                            goals_(_nr_robots),
                                            mrr_(_nr_robots),
                                            mrrTs_(_nr_robots, 8)
{
    robot_nr_ = _nr_robots;
    std::vector<uint32_t> robotRadius(robot_nr_, 0);
    setPlannerType(routerType::multiThreadSrr, 8);
}

void Router::setCollisionResolutionType(const SegmentExpander::CollisionResolverType _cr)
{
    multiRobotRouter_->setCollisionResolver(_cr);
}

void Router::setPlannerType(Router::routerType _type, uint32_t _nr_threads)
{
    if (_type == routerType::multiThreadSrr)
        multiRobotRouter_ = &mrrTs_;
    else
        multiRobotRouter_ = &mrr_;
}

void Router::resize(const uint32_t _nr_robots)
{
    robot_nr_ = _nr_robots;
    starts_.resize(_nr_robots);
    goals_.resize(_nr_robots);

    startSegments_.resize(_nr_robots);
    goalSegments_.resize(_nr_robots);

    voronoiGoals_.resize(_nr_robots);
    voronoiStart_.resize(_nr_robots);

    realGoals_.resize(_nr_robots);
    realStart_.resize(_nr_robots);
}

bool Router::preprocessEndpoints(const std::vector<float> &_radius, const float &resolution, const Eigen::Vector2d &origin, const std::vector<Segment> &_graph)
{
    for (uint32_t i = 0; i < goals_.size(); i++)
    {
        //Find Start and Goal Poses On the map
        realStart_[i] = {(starts_[i][0] - origin[0]) / resolution, (starts_[i][1] - origin[1]) / resolution};
        realGoals_[i] = {(goals_[i][0] - origin[0]) / resolution, (goals_[i][1] - origin[1]) / resolution};

        float radius = _radius[i];
        float d_start = pointExpander_.getDistanceToObstacle(realStart_[i]);
        ROS_DEBUG("Multi Robot Router: robot %i \"%s\" @  <%f, %f >", i, robot_names_[i].c_str(), starts_[i][0], starts_[i][1]);
        if ( d_start < radius / 2)
        {
            ROS_INFO("Multi Robot Router: Start of robot %i \"%s\" @  <%f, %f > is to close to an obstacle", i, robot_names_[i].c_str(), starts_[i][0], starts_[i][1]);
            return false;
        }
        float d_goal = pointExpander_.getDistanceToObstacle(realGoals_[i]);
        if (d_goal < radius / 2)
        {
            ROS_INFO("Multi Robot Router: Goal of robot%i \"%s\"  @  <%f, %f > is to close to an obstacle", i, robot_names_[i].c_str(), starts_[i][0], starts_[i][1]);
            return false;
        }
    }

    return true;
}

bool Router::processEndpointsExpander(const cv::Mat &_map, const std::vector<Segment> &_graph, const Eigen::Vector2d &_realStart, const Eigen::Vector2d &_realGoal, Eigen::Vector2d &_voronoiStart, Eigen::Vector2d &_voronoiGoal, uint32_t &_segmentStart, uint32_t &_segmentGoal, const uint32_t _diameter, const uint32_t _index) const
{

    int32_t segIdStart = -1;
    int32_t segIdGoal = -1;

    //check if start and goal pose have enough clearance to obstacles
    uint32_t size_x = _map.cols;
    uint32_t size_y = _map.rows;

    //No algorithm checks free space to obstacle because it is allready checked
    if (graphMode_ == graphType::voronoi)
    {
        //Find Start and Goal (with distance to Segment (more performance but robot has to be inside a segment)
        segIdStart = getSegment(_graph, _realStart);
        segIdGoal = getSegment(_graph, _realGoal);

        if (segIdStart != -1 && segIdGoal != -1)
        {
            _voronoiStart = (_graph[segIdStart].getStart() + _graph[segIdStart].getEnd()) / 2;
            _voronoiGoal = (_graph[segIdGoal].getStart() + _graph[segIdGoal].getEnd()) / 2;
        }
    }
    else // if(graphMode_ == graphType::random)
    {
        PointExpander _expander;
        _expander.initialize(_map);

        //Save all segment Points into map to find them using point Expander
        std::map<uint32_t, Eigen::Vector2d> points;

        for (const Segment &seg : _graph)
        {
            std::pair<uint32_t, Eigen::Vector2d> p(seg.getSegmentId(), (seg.getStart() + seg.getEnd()) / 2);
            points.insert(p);
        }

        //find Segment using Dijkstra (less performance but find segment for sure)
        std::vector<float> potential;
        potential.resize(size_x * size_y);
        float *p = &potential[0];
        //potential_.reset(new float[]);

        _expander.findGoalOnMap(_realStart, size_x * size_y, p, points, 0, _voronoiStart, segIdStart, _diameter / 2); //It is allready checked if there is enough free space

        std::vector<float> potential2;
        potential2.resize(size_x * size_y);
        float *p2 = &potential2[0];
        _expander.findGoalOnMap(_realGoal, size_x * size_y, p2, points, 0, _voronoiGoal, segIdGoal, _diameter / 2); //It is allready checked if there is enough free space
    }

    if (segIdStart == -1)
    {
        ROS_INFO("Multi Robot Router: Start of robot %i \"%s\" was not found", _index, robot_names_[_index].c_str());
        return false;
    }

    if (segIdGoal == -1)
    {
        ROS_INFO("Multi Robot Router: Goal of robot %i \"%s\" was not found", _index, robot_names_[_index].c_str());
        return false;
    }

    _segmentStart = segIdStart;
    _segmentGoal = segIdGoal;

    //Optimize found segments for errors
    if (!resolveSegment(_graph, _segmentStart, _realStart, _diameter, _segmentStart))
    {
        ROS_INFO("Multi Robot Router: Start of robot %i \"%s\" is not valid",  _index, robot_names_[_index].c_str());
        return false;
    }

    if (!resolveSegment(_graph, _segmentGoal, _realGoal, _diameter, _segmentGoal))
    {
        ROS_INFO("Multi Robot Router: Goal of robot %i \"%s\" is not valid",  _index, robot_names_[_index].c_str());
        return false;
    }

    return true;
}

bool Router::calculateStartPoints(const std::vector<float> &_radius, const cv::Mat &_map, const float &resolution, const Eigen::Vector2d &origin, const std::vector<Segment> &_graph)
{
    if (!preprocessEndpoints(_radius, resolution, origin, _graph))
        return false;

    std::vector<std::thread> t;
    t.resize(robot_nr_);

    std::vector<uint32_t> result;
    result.resize(robot_nr_);

    for (uint32_t i = 0; i < goals_.size(); i++)
    {
        uint32_t &res = result[i];
        Eigen::Vector2d &realS = realStart_[i];
        Eigen::Vector2d &realG = realGoals_[i];
        Eigen::Vector2d &voronS = voronoiStart_[i];
        Eigen::Vector2d &voronG = voronoiGoals_[i];
        uint32_t &segS = startSegments_[i];
        uint32_t &segG = goalSegments_[i];
        uint32_t diam = 2 * ((float)_radius[i]) / resolution;

        t[i] = std::thread(
            [this, &res, _map, _graph, realS, realG, &voronS, &voronG, &segS, &segG, diam, i]() {
                res = (uint32_t)processEndpointsExpander(_map, _graph, realS, realG, voronS, voronG, segS, segG, diam, i);
            });

        //             if(!processEndpointsExpander(_map, _graph, realStart_[i], realGoals_[i], voronoiStart_[i], voronoiGoals_[i], startSegments_[i], goalSegments_[i], 2 * ((float) _radius[i]) / resolution, i))
        //                 return false;
    }

    for (uint32_t i = 0; i < goals_.size(); i++)
    {
        t[i].join();
    }

    for (uint32_t i = 0; i < goals_.size(); i++)
    {

        if (!result[i])
            return false;
    }
    return true;
}

int32_t Router::getSegment(const std::vector<Segment> &_graph, const Eigen::Vector2d &_odom) const
{
    float minDist = FLT_MAX;
    int32_t segment = -1;

    //Select the segment which contains the robot center
    for (uint32_t i = 0; i < _graph.size(); i++)
    {
        float d = distanceToSegment(_graph[i], _odom);

        if (d < minDist && d <= _graph[i].width())
        {
            segment = i;
            minDist = d;
        }
    }

    return segment;
}

float Router::distanceToSegment(const Segment &_s, const Eigen::Vector2d &_p) const
{
    Eigen::Vector2d n = _s.getEnd() - _s.getStart();
    Eigen::Vector2d pa = _s.getStart() - _p;

    float c = n.dot(pa);

    // Closest point is a
    if (c > 0.0f)
        return std::sqrt(pa.dot(pa));

    Eigen::Vector2d bp = _p - _s.getEnd();

    // Closest point is b
    if (n.dot(bp) > 0.0f)
        return std::sqrt(bp.dot(bp));

    // Closest point is between a and b
    Eigen::Vector2d e = pa - n * (c / n.dot(n));

    return std::sqrt(e.dot(e));
}

bool Router::resolveSegment(const std::vector<Segment> &_graph, const uint32_t &_segId, const Eigen::Vector2d &_originPoint, const float &_diameter, uint32_t &_foundSeg) const
{
    const Segment seg = _graph[_segId];

    if ((seg.getPredecessors().size() == 0 || seg.getSuccessors().size() == 0) && seg.width() < _diameter)
    {
        //If we are on a leave Segment we are allowed to move the robot one segment in the graph if its radius is to big.
        //Because it can happen, that a leave segment has a triangular shape and thus wrong width value.
        std::vector<uint32_t> neighbours;

        if (seg.getPredecessors().size() != 0)
            neighbours = seg.getPredecessors();
        else
            neighbours = seg.getSuccessors();

        float dist = std::numeric_limits<float>::max();

        //Find the closest neighbour
        for (const uint32_t &neighbour : neighbours)
        {
            float ds_x = _originPoint[0] - _graph[neighbour].getStart()[0];
            float ds_y = _originPoint[1] - _graph[neighbour].getStart()[1];
            float de_x = _originPoint[0] - _graph[neighbour].getEnd()[0];
            float de_y = _originPoint[1] - _graph[neighbour].getEnd()[1];
            float d = (std::sqrt(ds_x * ds_x + ds_y * ds_y) + std::sqrt(de_x * de_x + de_y * de_y)) / 2;

            if (d < dist && _diameter <= _graph[neighbour].width())
            {
                _foundSeg = neighbour;
                dist = d;
            }
        }

        //Return only true if a valid Segment is found, where the radius of the robot is smaller
        //than the segment width
        if (_diameter <= _graph[_foundSeg].width())
            return true;
    }
    else if (_diameter <= seg.width())
    {
        //if the radius is smaller than the segment width a valid segment is found
        return true;
    }

    //No valid wait segment is found
    return false;
}

bool Router::makePlan(const std::vector<Eigen::Vector3d> &_starts, const std::vector<Eigen::Vector3d> &_goals, const std::vector<float> &_radius, const cv::Mat &_map, const float &_resolution, const Eigen::Vector2d &_origin, const std::vector<Segment> &_graph, const std::vector<std::string> &_robot_names)
{
    robot_names_ = _robot_names;
    auto t1 = std::chrono::high_resolution_clock::now();
    std::clock_t startcputime = std::clock();

    if (_goals.size() != _starts.size() || _goals.size() != _radius.size())
    {
        ROS_INFO("Multi Robot Router: Wrong nr of goals, starts or radii %lu %lu %lu", _starts.size(), goals_.size(), _radius.size());
        auto t2 = std::chrono::high_resolution_clock::now();
        duration_ = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
        return false;
    }

    //Set nr robots
    robot_nr_ = _goals.size();
    resize(robot_nr_);

    ROS_INFO("=========================================================");
    pointExpander_.initialize(_map);
    goals_ = _goals;
    starts_ = _starts;

    if (!calculateStartPoints(_radius, _map, _resolution, _origin, _graph))
    {
        ROS_INFO("Multi Robot Router: Failed to find Endpoints !!!");
        auto t2 = std::chrono::high_resolution_clock::now();
        duration_ = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
        return false;
    }

    multiRobotRouter_->setRobotNr(robot_nr_);
    std::vector<uint32_t> diameter;

    for (int i = 0; i < _radius.size(); i++)
    {
        diameter.push_back(2 * ((float)_radius[i]) / _resolution);
    }

    multiRobotRouter_->setRobotDiameter(diameter);
    multiRobotRouter_->setPriorityRescheduling(priorityRescheduling_);
    multiRobotRouter_->setSpeedRescheduling(speedRescheduling_);
    routingTable_.clear();

    if (!multiRobotRouter_->getRoutingTable(_graph, startSegments_, goalSegments_, routingTable_, routerTimeLimit_s_))
    {
        ROS_INFO("Multi Robot Router: Failed to find Routing Table !!!");
        auto t2 = std::chrono::high_resolution_clock::now();
        duration_ = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
        return false;
    }

    if (segmentOptimizations_)
        optimizePaths(_graph);

    postprocessRoutingTable();

    //DEBUG STATS
    longestPatLength_ = 0;
    overallPathLength_ = 0;

    for (std::vector<Checkpoint> &path : routingTable_)
    {
        float lengthPath = 0;

        for (Checkpoint &seg : path)
        {
            Eigen::Vector3d vec = (seg.end - seg.start);
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

void Router::optimizePaths(const std::vector<Segment> &_graph)
{
    if (routingTable_.size() > 1)
        return;

    if (routingTable_.size() > 2)
    {
        routingTable_[0].erase(routingTable_[0].begin(), routingTable_[0].begin() + 1);
        routingTable_[0].erase(routingTable_[0].end() - 1, routingTable_[0].end());
    }
}

void Router::postprocessRoutingTable()
{
    if (goalMode_ == goalMode::use_voronoi_goal)
    {
        for (int i = 0; i < routingTable_.size(); i++)
        {
            routingTable_[i].front().start[0] = voronoiStart_[i][0];
            routingTable_[i].front().start[1] = voronoiStart_[i][1];
            routingTable_[i].back().end[0] = voronoiGoals_[i][0];
            routingTable_[i].back().end[1] = voronoiGoals_[i][1];
            routingTable_[i].back().end[2] = goals_[i][2];
        }
    }
    else if (goalMode_ == goalMode::use_map_goal)
    {
        for (int i = 0; i < routingTable_.size(); i++)
        {
            routingTable_[i].front().start[0] = realStart_[i][0];
            routingTable_[i].front().start[1] = realStart_[i][1];
            routingTable_[i].back().end[0] = realGoals_[i][0];
            routingTable_[i].back().end[1] = realGoals_[i][1];
            routingTable_[i].back().end[2] = goals_[i][2];
        }
    }
    else
    {
        for (int i = 0; i < routingTable_.size(); i++)
        {
            routingTable_[i].back().end[2] = goals_[i][2];
        }
    }
}

const std::vector<Checkpoint> &Router::getRoute(const uint32_t _robot) const
{
    return routingTable_[_robot];
}

uint32_t Router::getDuration_ms() const
{
    return duration_;
}

float Router::getLongestPathLength() const
{
    return longestPatLength_;
}

float Router::getOverallPathLength() const
{
    return overallPathLength_;
}

uint32_t Router::getPriorityScheduleAttemps() const
{
    return multiRobotRouter_->getPriorityScheduleAttempts();
}

uint32_t Router::getSpeedScheduleAttemps() const
{
    return multiRobotRouter_->getSpeedScheduleAttempts();
}
} // namespace multi_robot_router
