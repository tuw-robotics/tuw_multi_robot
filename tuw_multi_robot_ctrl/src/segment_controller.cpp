#include "simple_velocity_controller/segment_controller.h"
#include <ros/ros.h>
#include <memory>
#include <cmath>


namespace velocity_controller
{
    SegmentController::SegmentController()
    {
        actual_cmd_ = run;
        goodId = tuw_multi_robot_msgs::RobotInfo::GOOD_EMPTY;
    }

    void SegmentController::setPath( std::shared_ptr< std::vector< PathPoint > > _path )
    {
        actualPreconditions.clear();
        path_ = _path;
		pathCounter_ = 0;
        plan_active = true;
        robot_status = tuw_multi_robot_msgs::RobotInfo::STATUS_DRIVING;
    }

    int SegmentController::getCount()
    {
		return pathCounter_ - 1;
    }


    void SegmentController::setGoalRadius( float r )
    {
        goal_radius_ = r;
    }

    void SegmentController::updatePrecondition( PathPrecondition pc )
    {
		if(actualPreconditions.size() <= pc.robot)
			actualPreconditions.resize(pc.robot + 1, 0);
		
		actualPreconditions[pc.robot] = pc.stepCondition;
    }

    
    bool SegmentController::checkPrecondition( PathPoint p )
    {
        for ( const auto & pc : p.precondition )
        {
            int count = 0;

            if ( actualPreconditions.size() > pc.robot )
            {
                count = actualPreconditions[pc.robot];
            }
            if(count <= pc.stepCondition)	//??? <= TODO
			{
				return false;
			}
        }
        return true;
    }


    bool SegmentController::checkGoal( PathPoint _odom )
    {
        if ( plan_active && actual_cmd_ != wait_step && checkPrecondition( (*path_)[pathCounter_] ) )
        {
            float dy = ( float )( _odom.y -  (*path_)[pathCounter_].y );
            float dx = ( float )( _odom.x -  (*path_)[pathCounter_].x );

            if ( sqrt( dx * dx + dy * dy ) < goal_radius_ )
            {
                if ( pathCounter_ < path_->size() )
                {
                    //ROS_INFO( "++" );
                    pathCounter_++;

                    if ( actual_cmd_ == step )
                        actual_cmd_ = wait_step;


                    if ( pathCounter_ >= path_->size() )
                    {
                        ROS_INFO( "Multi Robot Controller: goal reached" );
                        robot_status = tuw_multi_robot_msgs::RobotInfo::STATUS_DONE;
                        plan_active = false;
                    }
                }
            }
        }
    }


    void SegmentController::setPID( float _Kp, float _Ki, float _Kd )
    {
        Kp_ = _Kp;
        Kd_ = _Kd;
        Ki_ = _Ki;
    }

    void SegmentController::setState( state s )
    {
        actual_cmd_ = s;
    }


    void SegmentController::update( PathPoint _odom, float _delta_t )
    {
        checkGoal( _odom );

        if ( plan_active && actual_cmd_ != wait_step && actual_cmd_ != stop && checkPrecondition( (*path_)[pathCounter_] ))
        {
            float theta = atan2( _odom.y - (*path_)[pathCounter_].y, _odom.x - (*path_)[pathCounter_].x );
            float d_theta = normalizeAngle( _odom.theta - theta  + M_PI );

            float d_theta_comp = M_PI / 2 + d_theta;
            float distance_to_goal = sqrt( ( _odom.x - (*path_)[pathCounter_].x ) * ( _odom.x - (*path_)[pathCounter_].x ) + ( _odom.y - (*path_)[pathCounter_].y ) * ( _odom.y - (*path_)[pathCounter_].y ) );
            float turn_radius_to_goal = absolute( distance_to_goal / 2 / cos( d_theta_comp ) );


            float e = -d_theta;

            e_dot_ += e;

            w_ = Kp_ * e + Ki_ * e_dot_ * _delta_t + Kd_ * ( e - e_last_ ) / _delta_t;
            e_last_ = e;

            if ( w_ > max_w_ )
                w_ = max_w_;
            else if ( w_ < -max_w_ )
                w_ = -max_w_;

            v_ = max_v_;

            if ( absolute( d_theta ) > M_PI / 4 )         //If angle is bigger than 90deg the robot should turn on point
            {
                v_ = 0;
            }
            else if ( turn_radius_to_goal < 10 * distance_to_goal ) //If we have a small radius to goal we should turn with the right radius
            {
                float v_h = turn_radius_to_goal * absolute( w_ );

                if ( v_ > v_h )
                    v_ = v_h;
            }

            //ROS_INFO("(%f %f)(%f %f)", odom.x, odom.y, path_iterator->x, path_iterator->y);
            //ROS_INFO("d %f t %f r %f, v %f w %f", distance_to_goal, d_theta / M_PI * 180, turn_radius_to_goal, v_, w_);
        }
        else
        {
            v_ = 0;
            w_ = 0;
        }
    }

    float SegmentController::absolute( float _val )
    {
        if ( _val < 0 )
            _val = -_val;

        return _val;

    }

    float SegmentController::normalizeAngle( float _angle )
    {

        while ( _angle >  M_PI )
        {
            _angle -= 2 * M_PI;
        }

        while ( _angle < -M_PI )
        {
            _angle += 2 * M_PI;
        }

        return _angle;
    }


    void SegmentController::setSpeedParams( float _max_v, float _max_w )
    {
        max_v_ = _max_v;
        max_w_ = _max_w;
    }

    void SegmentController::getSpeed( float* _v, float* _w )
    {
        *_v = v_;
        *_w = w_;
    }

    int SegmentController::getStatus()
    {
        return robot_status;
    }

    void SegmentController::setOrderId(int orderId)
    {
        this->orderId = orderId;
    }
    void SegmentController::setGoodId(int goodId)
    {
        this->goodId = goodId;
    }

    int SegmentController::getGoodId()
    {
        return goodId;
    }
    void SegmentController::setOrderStatus(int orderStatus)
    {
        this->orderStatus = orderStatus;
    }
    int SegmentController::getOrderStatus()
    {
        return this->orderStatus;
    }

}
