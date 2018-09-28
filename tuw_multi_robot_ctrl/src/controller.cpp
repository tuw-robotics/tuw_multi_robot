#include "simple_velocity_controller/controller.h"
#include <ros/ros.h>
#include <memory>
#include <cmath>

namespace velocity_controller
{
Controller::Controller()
{
  actual_cmd_ = run;
  idx_path_target_point_ = 0;
  current_pose_ = PathPoint();
}

void Controller::setPath(std::shared_ptr<std::vector<PathPoint> > _path)
{
  path_ = *_path;
  idx_path_target_point_ = 0;
  plan_active = true;
}

void Controller::setGoalRadius(float r)
{
  goal_radius_ = r;
}

bool Controller::checkGoal(PathPoint _odom)
{
  if (plan_active && actual_cmd_ != wait_step)
  {
    const PathPoint &target = path_[idx_path_target_point_];
    float dy = (float)(_odom.y - target.y);
    float dx = (float)(_odom.x - target.x);
    if (sqrt(dx * dx + dy * dy) < goal_radius_)
    {
      if (idx_path_target_point_ < path_.size())
      {
        // ROS_INFO("++");
        idx_path_target_point_++;
        if (actual_cmd_ == step)
          actual_cmd_ = wait_step;

        if (idx_path_target_point_ == path_.size())
        {
          ROS_INFO("Multi Robot Controller: goal reached");
          robot_status = tuw_multi_robot_msgs::RobotInfo::STATUS_DONE;
          plan_active = false;
        }
      }
    }
  }
}

void Controller::setPID(float _Kp, float _Ki, float _Kd)
{
  Kp_ = _Kp;
  Kd_ = _Kd;
  Ki_ = _Ki;
}

size_t Controller::getProgress(){
    return idx_path_target_point_;
}

bool Controller::isActive(){
    return plan_active;
}

void Controller::setState(state s)
{
  actual_cmd_ = s;
}

void Controller::update(PathPoint _odom, float _delta_t)
{
  current_pose_ = _odom;
  checkGoal(_odom);

  if (plan_active && actual_cmd_ != wait_step && actual_cmd_ != stop)
  {
    const PathPoint &target = path_[idx_path_target_point_];
    float theta = atan2(_odom.y - target.y, _odom.x - target.x);
    float d_theta = normalizeAngle(_odom.theta - theta + M_PI);

    float d_theta_comp = M_PI / 2 + d_theta;
    float distance_to_goal = sqrt((_odom.x - target.x) * (_odom.x - target.x) +
                                  (_odom.y - target.y) * (_odom.y - target.y));
    float turn_radius_to_goal = absolute(distance_to_goal / 2 / cos(d_theta_comp));

    float e = -d_theta;

    e_dot_ += e;

    w_ = Kp_ * e + Ki_ * e_dot_ * _delta_t + Kd_ * (e - e_last_) / _delta_t;
    e_last_ = e;

    if (w_ > max_w_)
      w_ = max_w_;
    else if (w_ < -max_w_)
      w_ = -max_w_;

    v_ = max_v_;
    if (absolute(d_theta) > M_PI / 4)  // If angle is bigger than 90deg the robot should turn on point
    {
      v_ = 0;
    }
    else if (turn_radius_to_goal <
             10 * distance_to_goal)  // If we have a small radius to goal we should turn with the right radius
    {
      float v_h = turn_radius_to_goal * absolute(w_);
      if (v_ > v_h)
        v_ = v_h;
    }

    // ROS_INFO("(%f %f)(%f %f)", odom.x, odom.y, path_iterator->x, path_iterator->y);
    // ROS_INFO("d %f t %f r %f, v %f w %f", distance_to_goal, d_theta / M_PI * 180, turn_radius_to_goal, v_, w_);
  }
  else
  {
    v_ = 0;
    w_ = 0;
  }
}

float Controller::absolute(float _val)
{
  if (_val < 0)
    _val = -_val;

  return _val;
}

float Controller::normalizeAngle(float _angle)
{
  while (_angle > M_PI)
  {
    _angle -= 2 * M_PI;
  }
  while (_angle < -M_PI)
  {
    _angle += 2 * M_PI;
  }
  return _angle;
}

void Controller::setSpeedParams(float _max_v, float _max_w)
{
  max_v_ = _max_v;
  max_w_ = _max_w;
}

void Controller::getSpeed(float* _v, float* _w)
{
  *_v = v_;
  *_w = w_;
}

    int Controller::getStatus()
    {
        return robot_status;
    }

    void Controller::setOrderId(int orderId)
    {
        this->orderId = orderId;
    }
    void Controller::setGoodId(int goodId)
    {
        this->goodId = goodId;
    }

    int Controller::getGoodId()
    {
        return goodId;
    }
    void Controller::setOrderStatus(int orderStatus)
    {
        this->orderStatus = orderStatus;
    }
    int Controller::getOrderStatus()
    {
        return this->orderStatus;
    }
}
