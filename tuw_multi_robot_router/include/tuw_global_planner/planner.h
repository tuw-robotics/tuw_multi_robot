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
#include <opencv/cv.h>
// #include <tuw_global_planner/utils.h>
#include <tuw_global_planner/segment.h>
#include <tuw_global_planner/point_expander.h>
// #include <tuw_global_planner/segment_expander.h>
// #include <tuw_global_planner/traceback.h>
// #include <tuw_global_planner/backtracking_avoid_resolution.h>
// #include <tuw_global_planner/backtracking_resolution.h>
// #include <tuw_global_planner/path_generator.h>
// #include <tuw_global_planner/priority_scheduler.h>
// #include <tuw_global_planner/speed_scheduler.h>
// #include <tuw_global_planner/potential_calculator.h>
// #include <tuw_global_planner/velocity_calculator.h>

class Planner
{
    public:
        Planner(int _nr_robots);
        Planner();
        void updateRobotPose(int _robot_id, const Eigen::Vector2d &_pose);

        /**
        * @brief resizes the planner to a different nr of _nr_robots
        */
        void resize(int _nr_robots);
        /**
        * @brief updates the robot start positin (normally called from odom r[_robot_id]
        */
     public:  
        /**
        * @brief generates the plan from (Vertex[odom robotPose] to Vertex[_goals]
        * @param _radius a vector of the robots radius'
        * @param _map the grid_map used to find the start and goal segments of the path
        * @param _graph the full graph of the map used for planning the path
        */       
        bool makePlan(const std::vector< Eigen::Vector2d > &_goals, const std::vector<float> &_radius, const cv::Mat &_map, const float &_resolution, const Eigen::Vector2d &_origin, const std::vector<Segment> &_graph);
        /**
        * @brief returns the Graph as Point with the found potential used for synchronization
        */
//     public:         const std::vector< Potential_Point > &getPath(int _robot_id);
        /**
        * @brief returns the Graph as Path Vertex with preconditions for every sgement
        **/
//     public:         const std::vector< PathSegment > &getPathSeg(int _robot_id);
        /**
        * @brief returns true if a plann is found TODO deprecated?
        */
//     public:         bool gotPlan();

//     public:         const std::vector<float> &getVelocityProfile(int _robot_id);



     private:        
       bool calculateStartPoints(const std::vector<float> &_radius, const cv::Mat &_map, const float &resolution, const Eigen::Vector2d &origin, const std::vector<Segment> &_graph);
       //Calculate a segment
       int getSegment(const std::vector<Segment> &_graph, const Eigen::Vector2d &_pose);
       //Helper dist calculation
       float distanceToSegment(const Segment &_s, const Eigen::Vector2d &_p);
       //Checks if _seg is a leave of the graph and uses the closes neighbor as segment if the width of the leave is to small
       bool resolveSegment(const std::vector< Segment > &_graph, const int &_segId, const Point &_originPoint, const float &_radius, int &_foundSeg);
       
//     private:        bool getPaths(const std::vector<std::shared_ptr<Vertex>> &_graph, int &_actualRobot, const std::vector<int> &_priorities, const std::vector<float>& _speedList, int maxStepsPotExp);
//     private:        bool resolveSegment(const std::vector< std::shared_ptr< Vertex > >& _graph, const std::shared_ptr<Vertex>& _seg, const Point& _originPoint, float _radius, std::shared_ptr< Vertex > &_foundSeg);
//
//     private:        bool gotPlan_;
//     private:        std::vector<std::vector< Potential_Point > > paths_;
//     private:        std::vector<std::vector< PathSegment > > segPaths_;
//
//     private:        std::vector< Point > realGoals_;
//     private:        std::vector< Point > realStart_;
//     private:        std::vector<std::shared_ptr<Vertex>> startSegments_;
//     private:        std::vector<std::shared_ptr<Vertex>> goalSegments_;
//     private:        std::vector< int >  radius_;
//     private:        std::vector<std::vector<float>> velocityProfile_;
//
      
//
//     private:        
//     private:        std::unique_ptr<SegmentExpander> expander_;
//     private:        std::unique_ptr<Traceback> traceback_;
//     private:        std::shared_ptr<Path_Coordinator> path_querry_;
//     private:        std::shared_ptr<BacktrackingAvoidResolution> resolution_;
//     private:        std::shared_ptr<PotentialCalculator> pCalc_;
//     private:        std::vector<std::vector<int>> segCounter;
//     private:        std::unique_ptr<PathGenerator<Potential_Point>> path_generator_Point_;
//     private:        std::unique_ptr<PathGenerator<PathSegment>> path_generator_Segment_;
//     private:        std::unique_ptr<PriorityScheduler> priorityScheduler_;
//     private:        std::unique_ptr<SpeedScheduler> speedScheduler_;
//     private:        std::unique_ptr<VelocityCalculator> velocityCalc_;
//
//     private:        int speedScheduleAttemps_;
//     private:        int priorityScheduleAttemps_;
//     private:        int overallPathLength_;
//     private:        int longestPatLength_;
//     private:        int duration_;
//
//
//     public:         int getDuration_ms();
//     public:         int getOverallPathLength();
//     public:         int getLongestPathLength();
//     public:         int getPriorityScheduleAttemps();
//     public:         int getSpeedScheduleAttemps();
//
//
//     private:        
//     private:        
    public:         
      std::shared_ptr<float> potential_; 

    private:
        int robot_nr_;
        std::vector<bool> pose_received_;
        std::vector<Eigen::Vector2d> robot_poses_;
        std::vector<Eigen::Vector2d> goals_;
        std::vector<Eigen::Vector2d> realGoals_;
        std::vector<Eigen::Vector2d> realStart_;
        std::vector<Eigen::Vector2d> voronoiGoals_;
        std::vector<Eigen::Vector2d> voronoiStart_;
        std::vector<int> startSegments_;
        std::vector<int> goalSegments_;
        std::vector<int>  diameter_;
        
        std::unique_ptr<PointExpander> pointExpander_;
        
    protected:
        bool useGoalOnSegment_;
        bool allowEndpointOffSegment_;
        int optimizationSegmentNr_;
};

#endif // PLANNER_H
