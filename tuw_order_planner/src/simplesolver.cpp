#include "simplesolver.h"
#include <cmath>

namespace tuw_order_planner
{

float distance(float x1, float y1, float x2, float y2)
{
  return sqrt(pow(abs(x1 - x2), 2) + pow(abs(y1 - y2), 2));
}

struct distance_comp
{
  bool operator()(dist_pair& a, dist_pair& b) const
  {
    return a.distance < b.distance;
  }
};

std::vector<TransportPair> SimpleSolver::solve()
{
  std::vector<dist_pair> distance_pairs;

  // compute distance between current position of orders and robots
  for (int i = 0; i < orders_.size(); ++i)
  {
    std::map<int, geometry_msgs::Pose*>::iterator search_order_position = 
      order_positions_.find(i);

    if (search_order_position == order_positions_.end())
      // order position not known. ignore 
      continue;

    float order_position_x = search_order_position->second->position.x;
    float order_position_y = search_order_position->second->position.y;

    tuw_multi_robot_msgs::Order order = orders_.at(i);

    std::map<std::string, geometry_msgs::Pose*>::iterator search_robot = robots_.begin();
    while (search_robot != robots_.end())
    {
      std::string robot_name = search_robot->first;
      geometry_msgs::Pose* pose = search_robot->second;

      if (pose == nullptr)
        continue;

      float dist = distance(
          order_position_x, 
          order_position_y, 
          pose->position.x,
          pose->position.y);

      dist_pair pair;
      pair.distance = dist;
      pair.order_id = order.order_id;
      pair.robot_name = robot_name;

      distance_pairs.push_back(pair);

      *search_robot++;
    }
  }

  // sort by computed robot <-> order distance
  std::sort(distance_pairs.begin(), distance_pairs.end(), distance_comp());

  // take all robot<->order assignments without having duplicate orders/robots
  std::vector<int> consumed_orders;
  std::vector<std::string> consumed_robots;
  std::vector<TransportPair> plan;
  for (auto const& pair : distance_pairs)
  {
    if (std::find(consumed_orders.begin(), consumed_orders.end(), pair.order_id) != consumed_orders.end())
      continue;
    if (std::find(consumed_robots.begin(), consumed_robots.end(), pair.robot_name) != consumed_robots.end())
      continue;
    consumed_orders.push_back(pair.order_id);
    consumed_robots.push_back(pair.robot_name);
    TransportPair t_pair;
    t_pair.robot_name = pair.robot_name;
    t_pair.order_id = pair.order_id;
    plan.push_back(t_pair);
  }

  return plan;
}

}  // end namespace tuw_order_planner
