#ifndef SEGMENT_CONTROLLER_H
#define SEGMENT_CONTROLLER_H


// ROS
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>


namespace  velocity_controller
{

	typedef struct PathPrecondition_t
	{
		int robot;
		int stepCondition;
	}PathPrecondition;
	
    typedef struct PathPoint_t
    {
        float x;
        float y;
        float theta;
		std::vector<PathPrecondition> precondition;
    } PathPoint;


    typedef enum state_t
    {
        run,
        stop,
        step,
        wait_step
    } state;

    class SegmentController
    {

        public:
            SegmentController();
            void setPath ( std::shared_ptr<std::vector<PathPoint>> _path );
            void update ( PathPoint _odom, float _delta_t );
            void setSpeedParams ( float _max_v, float _max_w );
            void getSpeed ( float* _v, float* _w );
            void setGoalRadius ( float _r );
            void setPID ( float _Kp, float _Ki, float _Kd );
            void setState ( state s );
			void updatePrecondition( PathPrecondition pc);
			int getCount();
        private:
			bool checkPrecondition(PathPoint p);
            float normalizeAngle ( float _angle );
            float absolute ( float _val );
            bool checkGoal ( PathPoint _odom );
            std::shared_ptr<std::vector<PathPoint>> path_;
            float max_v_, max_w_;
            float v_, w_;
			int pathCounter_ = 0;
            bool plan_active = false;
            float e_last_ = 0;
            float e_dot_ = 0;
            float Kp_ = 5;
            float Kd_ = 1;
            float Ki_ = 0.0;
            float goal_radius_ = 0.25;

			std::vector<int> actualPreconditions;
			
            state actual_cmd_ = run;
    };

}

#endif // CONTROLLER_NODE_H
