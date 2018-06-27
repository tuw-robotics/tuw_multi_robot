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
#include <tuw_multi_robot_route_to_path/RobotStateObserver.h>

#include <math.h>
#include <limits.h>
#include <ros/ros.h>

namespace  tuw_multi_robot_route_to_path
{
    RobotStateObserver::RobotStateObserver()
    {
        currentStep = 0;
    }
    int RobotStateObserver::init(const std::vector<PathSegment> &_path)
    {
        currentStep = 0;
        path_ = _path;

        return currentStep;
    }
    int RobotStateObserver::getStep(const Eigen::Vector2d &_odom, bool &_changed)
    {
        float minDist = FLT_MAX;
        bool inSeg;
        int cstep = currentStep;

        for(int i = currentStep; i < path_.size(); i++)
        {
            float d = distanceToSegment(path_[i], _odom);

            if(d < minDist)
            {
                currentStep = i;
                minDist = d;
            }
        }

        _changed = cstep != currentStep;
        
        return currentStep;
    }

    float RobotStateObserver::distanceToSegment(PathSegment _s, Eigen::Vector2d _p)
    {
        Eigen::Vector2d n = _s.goal - _s.start;
        Eigen::Vector2d pa = _s.start - _p;

        float c = n.dot(pa);

        // Closest point is a
        if(c > 0.0f)
            return pa.dot(pa);

        Eigen::Vector2d bp = _p - _s.goal;

        // Closest point is b
        if(n.dot(bp) > 0.0f)
            return bp.dot(bp);

        // Closest point is between a and b
        Eigen::Vector2d e = pa - n * (c / n.dot(n));


        return e.dot(e);
    }
}

