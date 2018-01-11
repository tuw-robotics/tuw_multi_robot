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

#ifndef PATH_COORDINATOR_TIMELESS_H
#define PATH_COORDINATOR_TIMELESS_H

#include <tuw_global_planner/path_coordinator.h>
#include <tuw_global_planner/segment.h>




class Path_Coordinator_Timed :  public Path_Coordinator
{  
    class Timeline
    {
        typedef struct seg_occupation_t
        {
            int robot;
            float spaceOccupied;
            int startTime;
            int endTime;                          //-1 means forever
            bool mainSeg;
            seg_occupation_t(int _robot, float _spaceOcupied, int _startTime, int _endTime):robot(_robot), spaceOccupied(_spaceOcupied), startTime(_startTime), endTime(_endTime), mainSeg(true)
            {
            }
            seg_occupation_t(int _robot, float _spaceOcupied, int _startTime, int _endTime, bool _mainSeg):robot(_robot), spaceOccupied(_spaceOcupied), startTime(_startTime), endTime(_endTime), mainSeg(_mainSeg)
            {
            }
        }seg_occupation;
      
        public:         Timeline();

        public:         void reset(std::vector< std::shared_ptr<Segment> > _graph);
        public:         bool addSegment(int _startTime, int _endTime, std::shared_ptr<Segment> _seg, int _robotNr, int _robotSize, bool _mainSeg);
        public:         bool checkSegment(int _startTime, int _endTime, std::shared_ptr<Segment> _seg, int _robotNr, int _robotSize, int &_lastCollisionRobot);
        public:         bool addCrossingSegment(int _startTime, int _endTime, std::shared_ptr<Segment> _seg, int _robotNr, int _robotSize, bool _mainSeg);
        public:         bool checkCrossingSegment(int _startTime, int _endTime, std::shared_ptr<Segment> _seg, int _robotNr, int _robotSize, int &_lastCollisionRobot);
        public:         int findSegId(int robot, int timestep);
        public:         int getSize();
        public:         int getTimeUntilRobotOnSegment(int _robotNr, std::shared_ptr< Segment > _seg);
	public:		std::vector<std::pair<int,float>> getListOfRobotsHigherPrioritizedRobots(int _robot, std::shared_ptr<Segment>  _segment);
        
        private:        std::vector<std::vector<seg_occupation>> timeline_;
        private:        std::vector<std::vector<int>> robotSegments_;
        private:        std::vector<float> segmentSpace_;
        private:        int maxTime_ = 0;
    };
  
  
public:       Path_Coordinator_Timed();
public:       void reset(std::vector< std::shared_ptr<Segment> > _graph, int _nrRobots);
public:       bool addPath(std::vector<std::shared_ptr<Segment>> &_path, int radius_pixel);  
public:       bool checkSegment(std::shared_ptr<Segment> _next, int _startTime, int _endTime, int _radius_pixel, int &_collisionRobot, bool _ignoreGoal=false);  
public:       void setActive(int _robotNr);
public:       bool setGoalSegments(const std::vector<std::shared_ptr<Segment>> _goalSegments, const std::vector<int> radius);
public:       bool setStartSegments(const std::vector<std::shared_ptr<Segment>> _startSegments, const std::vector<int> radius);
public:       bool isGoal(std::shared_ptr<Segment> _seg);
public: 	std::shared_ptr<Segment>  getStart();
public: 	std::shared_ptr<Segment>  getEnd();
public:       int findSegNr(int _robot, int _potential);
public:       int findPotentialUntilRobotOnSegment(int _robot, std::shared_ptr< Segment > _segment);        //-1 means forever
public:		  std::vector<std::pair<int,float>> getListOfRobotsHigherPrioritizedRobots(int _robot, std::shared_ptr<Segment>  _segment);
public:		  const std::vector<int> &getNrOfRobotCollisions(int _robot);
public:	      void updateNrOfCollisions(int _collisionRobot,int _collisions);


private:        bool checkSegmentSingle(std::shared_ptr<Segment> _next, int _startTime, int _endTime, int _radius_pixel, int &_collisionRobot, bool _ignoreGoal);  

  
private:        std::vector<int> startIndex_;
private:        std::vector<int> endIndex_;
private:        std::shared_ptr< std::vector< int > > robotSize_;
private:        int activeRobot_;
private:        Timeline timeline_;
//private:        std::vector<std::vector<std::shared_ptr<Segment> > > segPaths_;
private:		std::vector<std::vector<int>> robotCollisions_;
private:        std::vector<std::shared_ptr<Segment>> goalSegments_;
private:        std::vector<std::shared_ptr<Segment>> startSegments_;

};

#endif // PATH_COORDINATOR_TIMELESS_H
