#ifndef TUW_MULTI_ROBOT_GOAL_HANDLER
#define TUW_MULTI_ROBOT_GOAL_HANDLER

#include <ros/ros.h>
#include "tuw_multi_robot_msgs/RobotGoalsArray.h"

/**
 * class to cover the ros communication
 **/
class GoalHandlerNode  {
public:
    enum Mode {
        READ = 0,
        WRITE = 1
    };
    
    GoalHandlerNode ( ros::NodeHandle & n, Mode mode );
    void callback ( const tuw_multi_robot_msgs::RobotGoalsArray& msg );
    void publish ();
private:
    ros::NodeHandle n_;
    ros::NodeHandle n_param_;
    ros::Subscriber sub_goals_;
    double      loop_rate_;                 /// paramter
    bool        time_now_;                  /// parameter
    bool        run_once_;                  /// parameter
    std::string file_name_;                 /// parameter
    ros::Publisher pub_goals_;
    tuw_multi_robot_msgs::RobotGoalsArray msg_;
    int counter_;

    void publishGoal ( );

};
#endif // TUW_MULTI_ROBOT_GOAL_HANDLER
