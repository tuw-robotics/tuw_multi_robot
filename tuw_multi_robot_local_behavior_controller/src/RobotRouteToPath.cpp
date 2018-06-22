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
#include <tuw_multi_robot_route_to_path/RobotRouteToPath.h>


namespace  tuw_multi_robot_route_to_path
{
    RobotRouteToPath::RobotRouteToPath(const int &_nr_of_robots, const int &_robot_nr)
    {
        nr_of_robots_ = _nr_of_robots;
        robot_nr_ = _robot_nr;
    }
    int RobotRouteToPath::init(const std::vector<SyncedPathPoint> &_path)
    {
        path_ = _path;
        endPublish_ = 0;

        return 0;
    }                                          //Returns sync Step
    std::vector<Eigen::Vector3d> RobotRouteToPath::updateSync(const std::vector<int> &_sync_steps, bool &_changed)
    {
        std::vector<Eigen::Vector3d> path;

        for(int i = 0; i < path_.size(); i++)
        {
            for(const PathPrecondition & pc : path_[i].sync)
            {
                if(_sync_steps[pc.robot_no] < pc.step)
                {
                    _changed = endPublish_ != i;
                    endPublish_ = i;
                    return path;
                }
            }

            path.emplace_back(path_[i].p);
        }

        _changed = endPublish_ != path_.size();
        endPublish_ = path_.size();
        return path;
    }

}