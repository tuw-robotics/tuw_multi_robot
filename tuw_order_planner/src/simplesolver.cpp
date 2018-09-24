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

  for (int i = 0; i < orders_.size(); ++i)
  {
    tuw_multi_robot_msgs::Order order = orders_.at(i);

    std::map<std::string, geometry_msgs::Pose*>::iterator search = robots_.begin();
    while (search != robots_.end())
    {
      std::string robot_name = search->first;
      geometry_msgs::Pose* pose = search->second;

      if (pose == nullptr)
        continue;

      float dist = distance(
          order.positions.at(0).position.x, 
          order.positions.at(0).position.y, 
          pose->position.x,
          pose->position.y);

      dist_pair pair;
      pair.distance = dist;
      pair.order_id = order.order_id;
      pair.robot_name = robot_name;

      distance_pairs.push_back(pair);

      *search++;
    }
  }

  std::sort(distance_pairs.begin(), distance_pairs.end(), distance_comp());

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
