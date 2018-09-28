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

#include <math.h>
#include <limits.h>
#include <ros/ros.h>
#include <tuw_multi_robot_route_to_path/RouteProgressMonitor.h>

namespace  tuw {

RouteProgressMonitor::Segment::Segment ( double x0, double y0, double x1, double y1, double width )
    : l ( x0, y0, x1, y1 )
    , width ( width )
    , state ( SEGMENT_STATE_AHEAD ){
}

RouteProgressMonitor::RouteProgressMonitor()
    : idx_active_segment_(0) {
}

void RouteProgressMonitor::init ( const tuw_multi_robot_msgs::Route &route ) {
    for ( size_t i = 0; i < route.segments.size(); i++ ) {
        auto &seg = route.segments[i];
        SegmentPtr s = SegmentPtr ( new RouteProgressMonitor::Segment ( seg.start.position.x, seg.start.position.y, seg.end.position.x, seg.end.position.y, seg.width ) );
        if(i == 0){
            s->state = SEGMENT_STATE_ACTIVE;
        }else {
            s->state = SEGMENT_STATE_AHEAD;
        }
        segments_.push_back ( s );
    }
    idx_active_segment_ = 0;
}


bool RouteProgressMonitor::finished(){
    return (segments_.back()->state == SEGMENT_STATE_INACTIVE);
}

int RouteProgressMonitor::getProgress () {
    return idx_active_segment_ - 1;
}

void RouteProgressMonitor::updateProgress ( const tuw::Point2D p ) {
    for ( auto segment: segments_ ) {
        segment->distance = segment->l.distanceTo ( p );
        segment->distance = p.distanceTo(segment->l.p1());
    }
    if ( segments_.empty() ) {
        return;
    }
    if ( segments_.size() <= 1 ) {
        segments_[0]->state = SEGMENT_STATE_ACTIVE;
    }
    for ( size_t i = 0; i < segments_.size()-1; i++ ) {
        SegmentPtr curr = segments_[i];
        SegmentPtr next = segments_[i+1];
        if ( ( curr->state == SEGMENT_STATE_ACTIVE ) && ( next->state == SEGMENT_STATE_AHEAD ) ) {
            double d_curr = curr->distance;
            double d_next = next->distance;
            if ( d_next <= d_curr) {
                curr->state = SEGMENT_STATE_INACTIVE;
                next->state = SEGMENT_STATE_ACTIVE;
                idx_active_segment_ = i+1;
            }
        }
    }
}

};
