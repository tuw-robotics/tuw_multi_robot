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

#ifndef PLANNER_H
#define PLANNER_H

#include <vector>
#include <memory>
#include <grid_map_core/GridMap.hpp>
#include <tuw_global_planner/utils.h>
#include <tuw_global_planner/segment.h>
#include <tuw_global_planner/point_expander.h>
#include <tuw_global_planner/segment_expander.h>
#include <tuw_global_planner/traceback.h>
#include <tuw_global_planner/backtracking_avoid_resolution.h>
#include <tuw_global_planner/backtracking_resolution.h>
#include <tuw_global_planner/path_generator.h>
#include <tuw_global_planner/priority_scheduler.h>
#include <tuw_global_planner/speed_scheduler.h>
#include <tuw_global_planner/potential_calculator.h>
#include <tuw_global_planner/velocity_calculator.h>

class Planner
{
public:			Planner(int _nr_robots);
public:			Planner();

/**
 * @brief resizes the planner to a different nr of _nr_robots
 */
public:			void resize(int _nr_robots);
/**
 * @brief updates the robot start positin (normally called from odom r[_robot_id] 
 */
public:			void updateRobotPose(int _robot_id, const Point &_pose);
/**
 * @brief generates the plan from (Segment[odom robotPose] to Segment[_goals]
 * @param _radius a vector of the robots radius'
 * @param _map the grid_map used to find the start and goal segments of the path
 * @param _graph the full graph of the map used for planning the path
 */
public:			bool makePlan(const std::vector< Point > &_goals, const std::vector<float> &_radius, const grid_map::GridMap &_map, const std::vector<std::shared_ptr<Segment>> &_graph);
/**
 * @brief returns the Graph as Point with the found potential used for synchronization 
 */
public:			const std::vector< Potential_Point > &getPath(int _robot_id);
/**
 * @brief returns the Graph as Path Segment with preconditions for every sgement
 **/
public:			const std::vector< PathSegment > &getPathSeg(int _robot_id);
/**
 * @brief returns if a plann is found TODO deprecated?
 */
public: 		bool gotPlan();

public:			const std::vector<float> &getVelocityProfile(int _robot_id);



//private:		bool generatePath(std::vector<std::shared_ptr<Segment>> &_path, int _index);	//TODO take care const???
private:		bool calculateStartPoints(const std::vector<float> _radius, const grid_map::GridMap &_map, const std::vector<std::shared_ptr<Segment>> &_graph);
private:		bool getPaths(const std::vector<std::shared_ptr<Segment>> &_graph, int &_actualRobot, const std::vector<int> &_priorities, const std::vector<float>& _speedList);
private:        	bool findSegment (const std::vector< std::shared_ptr< Segment > >& _graph, const Point& _segmentPoint, const Point& _originPoint, float _radius, std::shared_ptr< Segment > &_foundSeg);

private:		bool gotPlan_;
private:		std::vector<std::vector< Potential_Point > > paths_;
private:		std::vector<std::vector< PathSegment > > segPaths_;
private:		std::vector< Point > goals_;
private:		std::vector< Point > robot_poses_;

private:		std::vector< Point > voronoiGoals_;
private:		std::vector< Point > voronoiStart_;
private:		std::vector<std::shared_ptr<Segment>> startSegments_;
private:		std::vector<std::shared_ptr<Segment>> goalSegments_;
private:		std::vector< int >  radius_;
private:        	std::vector<std::vector<float>> velocityProfile_;

public: 		std::shared_ptr<float> potential_;	//DEBUG PUBLISH

private:        	std::unique_ptr<PointExpander> pointExpander_;
private:		std::unique_ptr<SegmentExpander> expander_;
private:		std::unique_ptr<Traceback> traceback_;
private:        	std::shared_ptr<Path_Coordinator> path_querry_;
private:        	std::shared_ptr<BacktrackingAvoidResolution> resolution_;
private:		std::shared_ptr<PotentialCalculator> pCalc_;
private:		std::vector<std::vector<int>> segCounter;
private:		std::unique_ptr<PathGenerator<Potential_Point>> path_generator_Point_;
private:		std::unique_ptr<PathGenerator<PathSegment>> path_generator_Segment_;
private:		std::unique_ptr<PriorityScheduler> priorityScheduler_;
private:		std::unique_ptr<SpeedScheduler> speedScheduler_;
private:		std::unique_ptr<VelocityCalculator> velocityCalc_;

private:		int speedScheduleAttemps_;
private:		int priorityScheduleAttemps_;
private:		int overallPathLength_;
private:		int longestPatLength_;
private:		int duration_;


public:			int getDuration_ms();
public:			int getOverallPathLength();
public:			int getLongestPathLength();
public:			int getPriorityScheduleAttemps();
public:			int getSpeedScheduleAttemps();
};

#endif // PLANNER_H
