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

#ifndef TUW_MULTI_ROBOT_ROUTE_PROGRESS_MONITOR_H
#define TUW_MULTI_ROBOT_ROUTE_PROGRESS_MONITOR_H


#include <memory>
#include <vector>
#include <tuw_multi_robot_msgs/Route.h>
#include <tuw_geometry/linesegment2d.h>

namespace  tuw
{
class RouteProgressMonitor
{
    static const int SEGMENT_STATE_AHEAD = 0;
    static const int SEGMENT_STATE_ACTIVE = 1;
    static const int SEGMENT_STATE_INACTIVE = 2;


    class Segment {
    public:
        Segment ( double x0, double y0, double x1, double y1, double width );
        tuw::LineSegment2D l; 
        double width;
        int state;
        double distance;
    };
    
    typedef std::shared_ptr< Segment> SegmentPtr;
    
public:
    RouteProgressMonitor();
    
    
    void init(const tuw_multi_robot_msgs::Route &route);
    void updateProgress(const tuw::Point2D p);
    int getProgress();
    bool finished();
    int idx_active_segment_;
    
    tuw_multi_robot_msgs::Route route_;
    std::vector<SegmentPtr> segments_;
};

}

#endif

