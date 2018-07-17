#include "simplesolver.h"
#include <cmath>

namespace tuw_order_planner {

float distance(float x1, float y1, float x2, float y2)
{
    return sqrt(
            pow(abs(x1-x2), 2) +
            pow(abs(y1-y2), 2));
}

struct comp {
    bool operator()(dist_pair &a, dist_pair &b) const {
        return a.distance < b.distance;
    }
};

std::vector<transport_pair> SimpleSolver::solve()
{
    std::vector<dist_pair> distance_pairs;

    for( int i=0; i<goods.size(); ++i )
    {
        tuw_multi_robot_msgs::Good good = goods.at(i);

        std::map<std::string, tuw::ros_msgs::Pose*>::iterator search = robots.begin();
        while (search != robots.end())
        {
            std::string robot_name = search->first;
            tuw::ros_msgs::Pose *pose = search->second;

            if ( pose == nullptr )
                continue;

            float dist = distance(good.positions.at(0).position.x, good.positions.at(0).position.y, pose->position.x, pose->position.y);

            dist_pair pair;
            pair.distance = dist;
            pair.good_id = good.good_id;
            pair.robot_name = robot_name;

            distance_pairs.push_back(pair);

            *search++;
        }
    }

    std::sort(distance_pairs.begin(), distance_pairs.end(), comp());

    std::vector<int> consumed_goods;
    std::vector<std::string> consumed_robots;
    std::vector<transport_pair> plan;
    for( auto const& pair : distance_pairs)
    {
        if(std::find(consumed_goods.begin(), consumed_goods.end(), pair.good_id) != consumed_goods.end())
            continue;
        if(std::find(consumed_robots.begin(), consumed_robots.end(), pair.robot_name) != consumed_robots.end())
            continue;
        consumed_goods.push_back(pair.good_id);
        consumed_robots.push_back(pair.robot_name);
        transport_pair t_pair;
        t_pair.robot_name = pair.robot_name;
        t_pair.good_id = pair.good_id;
        plan.push_back(t_pair);
    }

    return plan;
}

} // end namespace tuw_order_planner
