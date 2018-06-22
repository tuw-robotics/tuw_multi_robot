/* Copyright (c) 2017, TU Wien
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
      * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY TU Wien ''AS IS'' AND ANY
  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL TU Wien BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef TUW_ROBOT_STATE_OBSERVER_H
#define TUW_ROBOT_STATE_OBSERVER_H


#include <memory>
#include <vector>
#include <eigen3/Eigen/Dense>

namespace  tuw_multi_robot_route_to_path
{
    struct PathSegment
    {
        Eigen::Vector2d start;
        Eigen::Vector2d goal;
        float width;
    };

    class RobotStateObserver
    {
            //special class-member functions.
        public:     RobotStateObserver();
        public:     int init(const std::vector<PathSegment> &_path);
        public:     int getStep(const Eigen::Vector2d &_odom, bool &_changed);

        private:    float distanceToSegment(PathSegment _s, Eigen::Vector2d _p);

        private:    std::vector<PathSegment> path_;
        private:    int currentStep;
    };

}

#endif

