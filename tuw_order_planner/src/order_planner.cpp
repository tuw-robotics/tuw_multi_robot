#include "order_planner.h"
#include <regex>

#include <tuw_multi_robot_msgs/RobotGoals.h>
#include <tuw_multi_robot_msgs/RobotGoalsArray.h>
#include <tuw_multi_robot_msgs/RobotInfo.h>

namespace tuw_order_planner
{
OrderPlanner::OrderPlanner(int argc, char** argv)
{
  ros::init(argc, argv, "order_planner");
  nodeHandle_ = new ros::NodeHandle();

  mode_ = MODE_INIT;

  subscribers_.push_back(nodeHandle_->subscribe("/orders", 0, &OrderPlanner::ordersCallback, this));
  subscribers_.push_back(nodeHandle_->subscribe("/robot_info", 10, &OrderPlanner::robotInfoCallback, this));

  pub_robot_goals_ = nodeHandle_->advertise<tuw_multi_robot_msgs::RobotGoalsArray>("/goals", 0);
  pub_pickup_ = nodeHandle_->advertise<tuw_multi_robot_msgs::Pickup>("/pickup", 10);
  pub_order_position_ = nodeHandle_->advertise<tuw_multi_robot_msgs::OrderPosition>("/order_position", 10);
}

void OrderPlanner::run()
{
  ros::spin();
}

// robots_progress_ indicates which stations have already been reached within
// a robots current order. 
int OrderPlanner::incrementRobotProgress(std::string robot_name)
{
  std::map<std::string, int>::iterator search = robots_progress_.find(robot_name);
  int progress = 0;
  if (search != robots_status_.end())
  {
    tuw_multi_robot_msgs::Order* order = findOrderByRobotName(robot_name);
    if (order == nullptr)
      return -1;
    if (search->second < order->stations.size())
    {
      search->second++;
    }
    progress = search->second;
  }
  return progress;
}

// reset progress made by all robots,
// publish empty goals
// publish pickup empty goods for all known robots
void OrderPlanner::reset()
{
  orders_.clear();
  std::map<std::string, int>::iterator search = robots_progress_.begin();
  while (search != robots_progress_.end())
  {
    search->second = 0;
    ++search;
  }

  search = attached_orders_.begin();
  while (search != attached_orders_.end())
  {
    search->second = tuw_multi_robot_msgs::RobotInfo::GOOD_EMPTY;
    ++search;
  }

  std::map<std::string, geometry_msgs::Pose*>::iterator search_robot = subscribed_robots_.begin();
  while (search_robot != subscribed_robots_.end())
  {
    std::string robot_name = search_robot->first;
    publishPickup(robot_name, tuw_multi_robot_msgs::RobotInfo::GOOD_EMPTY);
    ++search_robot;
  }
  mode_ = MODE_INIT;

  // publish empty goals so all robots stop
  tuw_multi_robot_msgs::RobotGoalsArray goals_array;
  pub_robot_goals_.publish(goals_array);
}

void OrderPlanner::robotInfoCallback(const tuw_multi_robot_msgs::RobotInfo::ConstPtr& robotInfo)
{
  std::string robot_name = robotInfo->robot_name;

  float x = robotInfo->pose.pose.position.x;
  float y = robotInfo->pose.pose.position.y;
  float z = robotInfo->pose.pose.position.z;
  
  int status = robotInfo->status;
  int orderId = robotInfo->order_id;
  // dont care about order id from robotInfo here,
  // as we have also stored that information

  geometry_msgs::Pose* new_pose = new geometry_msgs::Pose();
  new_pose->position.x = x;
  new_pose->position.y = y;
  new_pose->position.z = z;

  // check if this robot is known already, and save its current pose.
  // publish the current position of the order, if this robot is currently
  // transporting
  std::map<std::string, geometry_msgs::Pose*>::iterator search_robot = 
    subscribed_robots_.find(robot_name);
  if (search_robot != subscribed_robots_.end())
  {
    search_robot->second = new_pose;

    std::map<std::string, int>::iterator attached_order = attached_orders_.find(robot_name);
    if (attached_order != attached_orders_.end())
    {
      int order_id = attached_order->second;
      if (order_id >= 0)
      {
        publishOrderPosition(order_id, robotInfo->pose.pose);
      }
    }
  }
  else
  {
    // if unknown, add to known robots
    subscribed_robots_.insert(std::map<std::string, geometry_msgs::Pose*>::value_type(robot_name, NULL));
  }


  // determine if we need to compute new /goals
  std::map<std::string, int>::iterator search = robots_status_.find(robot_name);
  if (search != robots_status_.end())
  {
    if (status == tuw_multi_robot_msgs::RobotInfo::STATUS_DONE)
    {
      mode_ = MODE_PROGRESS;

      int order_id = findOrderIdByRobotName(robot_name);
      tuw_multi_robot_msgs::Order* order = findOrderByOrderId(order_id);

      if (order != nullptr)
      {
        int progress = incrementRobotProgress(robot_name);

        if (progress == 1)
        {
          // robot reached first station, pick up order
          publishPickup(robot_name, order_id);

          // assign the order to the robot
          std::map<std::string, int>::iterator at_search = attached_orders_.find(robot_name);
          if (at_search != attached_orders_.end())
          {
            if (order_id != at_search->second)
              at_search->second = order_id;
          }
          else
          {
            attached_orders_.insert(std::map<std::string, int>::value_type(robot_name, order_id));
          }

          // compute next goals
          route();
        }
        else if (order_id >= 0 && order != nullptr && progress == order->stations.size())
        {
          // robot reached last station, drop order
          std::map<std::string, int>::iterator at_search = attached_orders_.find(robot_name);

          publishPickup(robot_name, tuw_multi_robot_msgs::RobotInfo::GOOD_EMPTY);
          at_search->second = tuw_multi_robot_msgs::RobotInfo::GOOD_EMPTY;

          // set position of order to position of last station
          geometry_msgs::Pose pose = order->stations.at(order->stations.size() - 1).pose;
          publishOrderPosition(order_id, pose);
        }
        else
        {
          // robot reached intermediate goal, compute next goals
          route();
        }
      }
    }

    // store robot status
    search->second = status;
  }
  else
  {
    // robot previusly unknown
    robots_status_.insert(std::map<std::string, int>::value_type(robot_name, status));
    robots_progress_.insert(std::map<std::string, int>::value_type(robot_name, 0));
  }
}

// gets called when user clicks on 'Start' in rqt_ordermanager
void OrderPlanner::ordersCallback(const tuw_multi_robot_msgs::OrderArray::ConstPtr& orders)
{
  // reset state, save orders
  reset();
  orders_ = orders->orders;

  // initially set all orders to their first station
  for (int i = 0; i < orders_.size(); ++i)
  {
    tuw_multi_robot_msgs::Order order = orders_.at(i);
    geometry_msgs::Pose pose = order.stations.at(0).pose;
    publishOrderPosition(order.order_id, pose);
  }

  // compute goals
  route();
}

// compute goals such that the next station in each order gets reached
// (called repeatedly)
void OrderPlanner::route()
{
  if (orders_.size() == 0)
    return;

  tuw_multi_robot_msgs::RobotGoalsArray goals_array;

  // in MODE_INIT, goals are published such that all robots drive to the 
  // first station of their order
  // in MODE_PROGRESS, goals are published such that all robots drive to the
  // next station of their order (which might be the same station for 
  // particular robots in subsequent calls)

  if (mode_ == MODE_INIT)
  {
    // find an assignment of robots to orders.
    AbstractSolver* solver = new SimpleSolver(
        subscribed_robots_, orders_, order_positions_);
    transport_pairs_ = solver->solve();

    // prepare to publish this assignment
    std::vector<std::string> consumed_robots;
    for (auto const& pair : transport_pairs_)
    {
      tuw_multi_robot_msgs::Order* order = findOrderByOrderId(pair.order_id);

      if (order == nullptr)
        continue;

      tuw_multi_robot_msgs::RobotGoals goals;
      geometry_msgs::Pose pose;
      pose.position.x = order->stations.at(0).pose.position.x;
      pose.position.y = order->stations.at(0).pose.position.y;
      pose.orientation.x = 0;
      pose.orientation.y = 0;
      pose.orientation.z = 0;
      pose.orientation.w = 1;

      goals.destinations.push_back(pose);
      goals.robot_name = pair.robot_name;

      goals_array.robots.push_back(goals);

      consumed_robots.push_back(pair.robot_name);
    }

    // assign unused robots to their current position
    // (MRRP might have difficulties otherwise)
    std::map<std::string, geometry_msgs::Pose*>::iterator search = 
      subscribed_robots_.begin();

    while (search != subscribed_robots_.end())
    {
      std::string robot_name = search->first;
      geometry_msgs::Pose* pose = search->second;

      if (pose != nullptr &&
          std::find(
            consumed_robots.begin(), 
            consumed_robots.end(), 
            robot_name) == 
          consumed_robots.end())
      {
        tuw_multi_robot_msgs::RobotGoals goals;
        goals.destinations.push_back(*pose);
        goals.robot_name = robot_name;
        goals_array.robots.push_back(goals);
      }
      ++search;
    }

    // publish assignment
    pub_robot_goals_.publish(goals_array);
  }
  else if (mode_ == MODE_PROGRESS)
  {
    // for all robots, set the station next up in the order as goal

    std::map<std::string, int>::iterator r = robots_status_.begin();

    // finished counts how many robots have reached their last station
    int finished = 0;

    std::vector<std::string> consumed_robots;
    while (r != robots_status_.end())
    {
      std::string robot_name = r->first;
      int robot_status = r->second;
      std::map<std::string, int>::iterator search = robots_progress_.find(robot_name);
      int progress = search->second;

      tuw_multi_robot_msgs::Order* order = findOrderByRobotName(robot_name);

      if (order != nullptr)
      {
        tuw_multi_robot_msgs::RobotGoals goals;
        geometry_msgs::Pose pose;

        // TODO: robots may have moved since the last robotInfo message, which
        // is not respected here

        if (progress < order->stations.size())
        {
          pose.position.x = order->stations.at(progress).pose.position.x;
          pose.position.y = order->stations.at(progress).pose.position.y;
        }
        else
        {
          pose.position.x = order->stations.at(progress - 1).pose.position.x;
          pose.position.y = order->stations.at(progress - 1).pose.position.y;

          ++finished;
        }

        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0;
        pose.orientation.w = 1;
        goals.destinations.push_back(pose);
        goals.robot_name = robot_name;
        goals_array.robots.push_back(goals);

        consumed_robots.push_back(robot_name);
      }

      ++r;
    }

    // assign unused robots
    std::map<std::string, geometry_msgs::Pose*>::iterator search = subscribed_robots_.begin();
    while (search != subscribed_robots_.end())
    {
      std::string robot_name = search->first;
      geometry_msgs::Pose* pose = search->second;

      if (pose != nullptr &&
          std::find(consumed_robots.begin(), consumed_robots.end(), robot_name) == consumed_robots.end())
      {
        tuw_multi_robot_msgs::RobotGoals goals;
        goals.destinations.push_back(*pose);
        goals.robot_name = robot_name;
        goals_array.robots.push_back(goals);
      }
      ++search;
    }

    if (finished == robots_status_.size())
    {
      // when all robots are finished, reset state
      reset();
    }
    else
    {
      // otherwise, publish goals
      pub_robot_goals_.publish(goals_array);
    }
  } // end MODE_PROGRESS
}

tuw_multi_robot_msgs::Order* OrderPlanner::findOrderByRobotName(std::string robot_name)
{
  return findOrderByOrderId(findOrderIdByRobotName(robot_name));
}

tuw_multi_robot_msgs::Order* OrderPlanner::findOrderByOrderId(int id)
{
  if (id < 0)
    return nullptr;
  for (int i = 0; i < orders_.size(); ++i)
  {
    tuw_multi_robot_msgs::Order* order = &(orders_.at(i));
    if (order->order_id == id)
      return order;
  }
  return nullptr;
}

int OrderPlanner::findOrderIdByRobotName(std::string robot_name)
{
  int order_id;
  for (auto const& pair : transport_pairs_)
  {
    if (robot_name == pair.robot_name)
    {
      return pair.order_id;
    }
  }
  return -1;
}

// publishes the current position of an order such that it can be visualized
// saves this position in order_positions_
void OrderPlanner::publishOrderPosition(int order_id, geometry_msgs::Pose pose)
{
  tuw_multi_robot_msgs::OrderPosition orderPosition;
  orderPosition.order_id = order_id;
  orderPosition.position = pose;
  pub_order_position_.publish(orderPosition);
    
  geometry_msgs::Pose* new_pose = new geometry_msgs::Pose();
  new_pose->position.x = pose.position.x;
  new_pose->position.y = pose.position.y;
  new_pose->position.z = pose.position.z;

  std::map<int, geometry_msgs::Pose*>::iterator search_order_position = 
    order_positions_.find(order_id);
  if (search_order_position == order_positions_.end())
    order_positions_.insert(std::map<int, geometry_msgs::Pose*>::value_type(order_id, new_pose));
  else
    search_order_position->second = new_pose;
}

// publish pickup message
void OrderPlanner::publishPickup(std::string robot_name, int order_id)
{
  tuw_multi_robot_msgs::Pickup pickup;
  pickup.robot_name = robot_name;
  pickup.order_id = order_id;
  pub_pickup_.publish(pickup);
}

}  // end namespace tuw_order_planner

int main(int argc, char** argv)
{
  tuw_order_planner::OrderPlanner* controller = new tuw_order_planner::OrderPlanner(argc, argv);
  controller->run();

  return 0;
}
