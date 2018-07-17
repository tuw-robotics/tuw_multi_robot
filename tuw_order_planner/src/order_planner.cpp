#include "order_planner.h"
#include <regex>

#include <tuw_multi_robot_msgs/RobotGoals.h>
#include <tuw_multi_robot_msgs/RobotGoalsArray.h>
#include <tuw_multi_robot_msgs/RobotInfo.h>

namespace tuw_order_planner
{
OrderPlanner::OrderPlanner(int argc, char** argv)
{
  mode_ = MODE_INIT;
  ros::init(argc, argv, "order_planner");
  nodeHandle_ = new ros::NodeHandle();
  subscribers_.push_back(nodeHandle_->subscribe("/orders", 0, &OrderPlanner::ordersCallback, this));
  subscribers_.push_back(nodeHandle_->subscribe("/robot_info", 10, &OrderPlanner::robotInfoCallback, this));
  subscribeRobotOdom();
  pub_robot_goals_ = nodeHandle_->advertise<tuw_multi_robot_msgs::RobotGoalsArray>("goals", 0);
  pub_pickup_ = nodeHandle_->advertise<tuw_multi_robot_msgs::Pickup>("pickup", 10);
  pub_good_position_ = nodeHandle_->advertise<tuw_multi_robot_msgs::GoodPosition>("good_position", 10);
}

void OrderPlanner::run()
{
  ros::spin();
}

int OrderPlanner::incrementRobotProgress(std::string robot_name)
{
  std::map<std::string, int>::iterator search = robots_progress_.find(robot_name);
  int progress = 0;
  if (search != robots_status_.end())
  {
    tuw_multi_robot_msgs::Order* order = findOrderByRobotName(robot_name);
    if (order == nullptr)
      return -1;
    if (search->second < order->positions.size())
    {
      search->second++;
    }
    progress = search->second;
  }
  return progress;
}

void OrderPlanner::reset()
{
  orders_.clear();
  std::map<std::string, int>::iterator search = robots_progress_.begin();
  while (search != robots_progress_.end())
  {
    search->second = 0;
    ++search;
  }

  search = attached_goods_.begin();
  while (search != attached_goods_.end())
  {
    search->second = tuw_multi_robot_msgs::RobotInfo::GOOD_EMPTY;
    ++search;
  }
  mode_ = MODE_INIT;
}

void OrderPlanner::robotInfoCallback(const tuw_multi_robot_msgs::RobotInfo::ConstPtr& robotInfo)
{
  std::string robot_name = robotInfo->robot_name;
  int status = robotInfo->status;
  int goodId = robotInfo->good_id;
  // dont care about good id from robot_info here

  std::map<std::string, geometry_msgs::Pose*>::iterator active_robot = subscribed_robots_.find(robot_name);
  if (active_robot == subscribed_robots_.end())
    subscribed_robots_.insert(std::map<std::string, geometry_msgs::Pose*>::value_type(robot_name, NULL));

  std::map<std::string, int>::iterator search = robots_status_.find(robot_name);

  if (search != robots_status_.end())
  {
    if (status != search->second)
    {
      if (status == tuw_multi_robot_msgs::RobotInfo::STATUS_DONE)
      {
        mode_ = MODE_PROGRESS;

        int good_id = findGoodIdByRobotName(robot_name);
        tuw_multi_robot_msgs::Order* order = findOrderByGoodId(good_id);

        if (order != nullptr)
        {
          int progress = incrementRobotProgress(robot_name);

          if (progress == 1)
          {
            // robot pick up good

            publishPickup(robot_name, good_id);

            std::map<std::string, int>::iterator at_search = attached_goods_.find(robot_name);
            if (at_search != attached_goods_.end())
            {
              if (good_id != at_search->second)
                at_search->second = good_id;
            }
            else
            {
              attached_goods_.insert(std::map<std::string, int>::value_type(robot_name, good_id));
            }

            route();
          }
          else if (good_id >= 0 && order != nullptr && progress == order->positions.size())
          {
            // robot drop good

            std::map<std::string, int>::iterator at_search = attached_goods_.find(robot_name);

            publishPickup(robot_name, tuw_multi_robot_msgs::RobotInfo::GOOD_EMPTY);
            at_search->second = tuw_multi_robot_msgs::RobotInfo::GOOD_EMPTY;

            geometry_msgs::Pose pose = order->positions.at(order->positions.size() - 1);
            publishGoodPosition(good_id, pose);
          }
          else
          {
            // intermediate goal
            route();
          }
        }
      }
      search->second = status;
    }
  }
  else
  {
    robots_status_.insert(std::map<std::string, int>::value_type(robot_name, status));
    robots_progress_.insert(std::map<std::string, int>::value_type(robot_name, 0));
  }
}

void OrderPlanner::ordersCallback(const tuw_multi_robot_msgs::OrderArray::ConstPtr& orders)
{
  reset();
  orders_ = orders->orders;

  for (int i = 0; i < orders_.size(); ++i)
  {
    tuw_multi_robot_msgs::Order order = orders_.at(i);
    geometry_msgs::Pose pose = order.positions.at(0);
    publishGoodPosition(order.good_id, order.positions.at(0));
  }

  route();
}

void OrderPlanner::route()
{
  if (orders_.size() == 0)
    return;

  tuw_multi_robot_msgs::RobotGoalsArray goals_array;
  if (mode_ == MODE_INIT)
  {
    AbstractSolver* solver = new SimpleSolver(subscribed_robots_, orders_);
    transport_pairs_ = solver->solve();

    std::vector<std::string> consumed_robots;
    for (auto const& pair : transport_pairs_)
    {
      tuw_multi_robot_msgs::Order* order = findOrderByGoodId(pair.good_id);

      if (order == nullptr)
        continue;

      tuw_multi_robot_msgs::RobotGoals goals;
      geometry_msgs::Pose pose;
      pose.position.x = order->positions.at(0).position.x;
      pose.position.y = order->positions.at(0).position.y;
      pose.orientation.x = 0;
      pose.orientation.y = 0;
      pose.orientation.z = 0;
      pose.orientation.w = 1;

      goals.path_points.push_back(pose);
      goals.robot_name = pair.robot_name;

      goals_array.goals.push_back(goals);

      consumed_robots.push_back(pair.robot_name);
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
        goals.path_points.push_back(*pose);
        goals.robot_name = robot_name;
        goals_array.goals.push_back(goals);
      }
      ++search;
    }

    pub_robot_goals_.publish(goals_array);
  }
  else if (mode_ == MODE_PROGRESS)
  {
    std::map<std::string, int>::iterator r = robots_status_.begin();
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

        if (progress < order->positions.size())
        {
          pose.position.x = order->positions.at(progress).position.x;
          pose.position.y = order->positions.at(progress).position.y;
        }
        else
        {
          pose.position.x = order->positions.at(progress - 1).position.x;
          pose.position.y = order->positions.at(progress - 1).position.y;

          ++finished;
        }

        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0;
        pose.orientation.w = 1;
        goals.path_points.push_back(pose);
        goals.robot_name = robot_name;
        goals_array.goals.push_back(goals);

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
        goals.path_points.push_back(*pose);
        goals.robot_name = robot_name;
        goals_array.goals.push_back(goals);
      }
      ++search;
    }

    if (finished == robots_status_.size())
    {
      // reset();
    }
    else
    {
      pub_robot_goals_.publish(goals_array);
    }
  }
}

void OrderPlanner::subscribeRobotOdom()
{
  ros::master::V_TopicInfo master_topics;
  ros::master::getTopics(master_topics);

  for (ros::master::V_TopicInfo::iterator it = master_topics.begin(); it != master_topics.end(); it++)
  {
    const ros::master::TopicInfo& info = *it;
    if (std::regex_match(info.name, std::regex("/robot_(\\d)+/odom")))
    {
      subscribers_.push_back(nodeHandle_->subscribe(info.name, 1, &OrderPlanner::odomCallback, this));
    }
  }
}

void OrderPlanner::odomCallback(const nav_msgs::Odometry& odom)
{
  std::string robot_name = odom.header.frame_id.substr(1, odom.header.frame_id.find_last_of('/') - 1);

  float x = odom.pose.pose.position.x;
  float y = odom.pose.pose.position.y;
  float z = odom.pose.pose.position.z;

  geometry_msgs::Pose* new_pose = new geometry_msgs::Pose();
  new_pose->position.x = x;
  new_pose->position.y = y;
  new_pose->position.z = z;

  std::map<std::string, geometry_msgs::Pose*>::iterator search = subscribed_robots_.find(robot_name);

  if (search == subscribed_robots_.end())
    return;

  search->second = new_pose;

  std::map<std::string, int>::iterator attached_good = attached_goods_.find(robot_name);
  if (attached_good != attached_goods_.end())
  {
    int good_id = attached_good->second;
    if (good_id >= 0)
    {
      publishGoodPosition(good_id, odom.pose.pose);
    }
  }
}

tuw_multi_robot_msgs::Order* OrderPlanner::findOrderByRobotName(std::string robot_name)
{
  return findOrderByGoodId(findGoodIdByRobotName(robot_name));
}

tuw_multi_robot_msgs::Order* OrderPlanner::findOrderByGoodId(int id)
{
  if (id < 0)
    return nullptr;
  for (int i = 0; i < orders_.size(); ++i)
  {
    tuw_multi_robot_msgs::Order* order = &(orders_.at(i));
    if (order->good_id == id)
      return order;
  }
  return nullptr;
}

int OrderPlanner::findGoodIdByRobotName(std::string robot_name)
{
  int good_id;
  for (auto const& pair : transport_pairs_)
  {
    if (robot_name == pair.robot_name)
    {
      return pair.good_id;
    }
  }
  return -1;
}

void OrderPlanner::publishGoodPosition(int good_id, geometry_msgs::Pose pose)
{
  tuw_multi_robot_msgs::GoodPosition goodPosition;
  goodPosition.good_id = good_id;
  goodPosition.position = pose;
  pub_good_position_.publish(goodPosition);
}

void OrderPlanner::publishPickup(std::string robot_name, int good_id)
{
  tuw_multi_robot_msgs::Pickup pickup;
  pickup.robot_name = robot_name;
  pickup.good_id = good_id;
  pub_pickup_.publish(pickup);
}

}  // end namespace tuw_order_planner

int main(int argc, char** argv)
{
  tuw_order_planner::OrderPlanner* controller = new tuw_order_planner::OrderPlanner(argc, argv);
  controller->run();

  return 0;
}
