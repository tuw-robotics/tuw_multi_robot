#include "order_manager.h"
#include <regex>

#include <tuw_multi_robot_msgs/RobotGoals.h>
#include <tuw_multi_robot_msgs/RobotGoalsArray.h>
#include <tuw_multi_robot_msgs/RobotInfo.h>

namespace tuw_order_manager {

OrderManager::OrderManager(int argc, char** argv)
{
    mode = MODE_INIT;
    ros::init(argc, argv, "order_manager");
    nodeHandle = new ros::NodeHandle();
    subscribers.push_back( nodeHandle->subscribe("/tuw_rqt_goods/goods", 0, &OrderManager::goodsCallback, this) );
    subscribers.push_back( nodeHandle->subscribe("/robot_info", 10, &OrderManager::robotInfoCallback, this) );
    subscribe_robot_odom();
    pub_robot_goals = nodeHandle->advertise<tuw_multi_robot_msgs::RobotGoalsArray>("goals", 0);
    pub_pickup = nodeHandle->advertise<tuw_multi_robot_msgs::Pickup>("pickup", 10);
    pub_good_pose = nodeHandle->advertise<tuw_multi_robot_msgs::GoodPosition>("good_pose", 10);
}

void OrderManager::run()
{
    ros::spin();
}



int OrderManager::increment_robot_progress(std::string robot_name)
{
    std::map<std::string, int>::iterator search = robots_progress.find(robot_name);
    int progress = 0;
    if ( search != robots_status.end() )
    {
        tuw_multi_robot_msgs::Good *good = findGoodByRobotName(robot_name);
        if ( good == nullptr )
            return -1;
        if ( search->second < good->positions.size() ) {
            search->second++;
        }
        progress = search->second;
    }
    return progress;
}

void OrderManager::reset()
{
    goods.clear();
    std::map<std::string, int>::iterator search = robots_progress.begin();
    while( search != robots_progress.end() )
    {
        search->second = 0;
        ++search;
    }

    search = attached_goods.begin();
    while( search != attached_goods.end() )
    {
        search->second = tuw_multi_robot_msgs::RobotInfo::GOOD_EMPTY;
        ++search;
    }
    mode = MODE_INIT;
}

void OrderManager::robotInfoCallback(const tuw_multi_robot_msgs::RobotInfo::ConstPtr& robotInfo)
{
    std::string robot_name = robotInfo->robot_name;
    int status = robotInfo->status;
    int goodId = robotInfo->good_id;
    // dont care about good id from robot_info here

    std::map<std::string, tuw::ros_msgs::Pose*>::iterator active_robot = subscribed_robots.find(robot_name);
    if ( active_robot == subscribed_robots.end() )
        subscribed_robots.insert(std::map<std::string, tuw::ros_msgs::Pose*>::value_type(robot_name, NULL));

    std::map<std::string, int>::iterator search = robots_status.find(robot_name);

    if ( search != robots_status.end() )
    {
        if ( status != search->second )
        {
            if ( status == tuw_multi_robot_msgs::RobotInfo::STATUS_DONE )
            {
                mode = MODE_PROGRESS;

                int good_id = findGoodIdByRobotName(robot_name);
                tuw_multi_robot_msgs::Good *good = findGoodByGoodId(good_id);

                if ( good != nullptr ) {

                    int progress = increment_robot_progress(robot_name);

                    if (progress == 1)
                    {
                        // robot pick up good

                        publishPickup(robot_name, good_id);

                        std::map<std::string, int>::iterator at_search = attached_goods.find(robot_name);
                        if ( at_search != attached_goods.end() )
                        {
                            if ( good_id != at_search->second )
                                at_search->second = good_id;
                        }
                        else
                        {
                            attached_goods.insert(std::map<std::string, int>::value_type(robot_name, good_id));
                        }

                        route();
                    }
                    else if ( good_id >= 0 && good != nullptr && progress == good->positions.size() )
                    {
                        // robot drop good

                        std::map<std::string, int>::iterator at_search = attached_goods.find(robot_name);

                        publishPickup(robot_name, tuw_multi_robot_msgs::RobotInfo::GOOD_EMPTY);
                        at_search->second = tuw_multi_robot_msgs::RobotInfo::GOOD_EMPTY;

                        geometry_msgs::Pose pose = good->positions.at(good->positions.size()-1);
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
        robots_status.insert(std::map<std::string, int>::value_type(robot_name, status));
        robots_progress.insert(std::map<std::string, int>::value_type(robot_name, 0));
    }
}

void OrderManager::goodsCallback(const tuw_multi_robot_msgs::Goods::ConstPtr& goods)
{
    reset();
    this->goods = goods->goods;

    for( int i=0; i<this->goods.size(); ++i ) 
    {
        tuw_multi_robot_msgs::Good good = this->goods.at(i);
        geometry_msgs::Pose pose = good.positions.at(0);
        publishGoodPosition(good.good_id, good.positions.at(0));
    }

    route();
}

void OrderManager::route()
{
    if ( goods.size() == 0 )
        return;

    tuw_multi_robot_msgs::RobotGoalsArray goals_array;
    if ( mode == MODE_INIT )
    {
        AbstractSolver *solver = new SimpleSolver(subscribed_robots, goods);
        std::vector<transport_pair> plan = solver->solve();
        transport_pairs = plan;

        std::vector<std::string> consumed_robots;
        for( auto const& pair : plan)
        {
            tuw_multi_robot_msgs::Good *good = findGoodByGoodId(pair.good_id);

            if ( good == nullptr )
                continue;

            tuw_multi_robot_msgs::RobotGoals goals;
            geometry_msgs::Pose pose;
            pose.position.x = good->positions.at(0).position.x;
            pose.position.y = good->positions.at(0).position.y;
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
        std::map<std::string, tuw::ros_msgs::Pose*>::iterator search = subscribed_robots.begin();
        while (search != subscribed_robots.end())
        {
            std::string robot_name = search->first;
            tuw::ros_msgs::Pose *pose = search->second;

            if(pose != nullptr && std::find(consumed_robots.begin(), consumed_robots.end(), robot_name) == consumed_robots.end())
            {
                tuw_multi_robot_msgs::RobotGoals goals;
                goals.path_points.push_back(*pose);
                goals.robot_name = robot_name;
                goals_array.goals.push_back(goals);
            }
            ++search;
        }


        pub_robot_goals.publish(goals_array);

    }
    else if ( mode == MODE_PROGRESS )
    {

        std::map<std::string, int>::iterator r = robots_status.begin();
        int finished = 0;
        std::vector<std::string> consumed_robots;
        while( r != robots_status.end() )
        {
            std::string robot_name = r->first;
            int robot_status = r->second;
            std::map<std::string, int>::iterator search = robots_progress.find(robot_name);
            int progress = search->second;

            tuw_multi_robot_msgs::Good *good = findGoodByRobotName(robot_name);

            if ( good != nullptr )
            {

                tuw_multi_robot_msgs::RobotGoals goals;
                geometry_msgs::Pose pose;

                if ( progress < good->positions.size() )
                {
                    pose.position.x = good->positions.at(progress).position.x;
                    pose.position.y = good->positions.at(progress).position.y;
                }
                else
                {
                    pose.position.x = good->positions.at(progress-1).position.x;
                    pose.position.y = good->positions.at(progress-1).position.y;

                    ++finished;
                }

                pose.orientation.x = 0;
                pose.orientation.y = 0;
                pose.orientation.z = 0;
                pose.orientation.w = 1;
                goals.path_points.push_back(pose);
                goals.robot_name = robot_name;
                goals_array.goals.push_back(goals);

                consumed_robots.push_back( robot_name );
            }

            ++r;

        }

        // assign unused robots
        std::map<std::string, tuw::ros_msgs::Pose*>::iterator search = subscribed_robots.begin();
        while (search != subscribed_robots.end())
        {
            std::string robot_name = search->first;
            tuw::ros_msgs::Pose *pose = search->second;

            if(pose != nullptr && std::find(consumed_robots.begin(), consumed_robots.end(), robot_name) == consumed_robots.end())
            {
                tuw_multi_robot_msgs::RobotGoals goals;
                goals.path_points.push_back(*pose);
                goals.robot_name = robot_name;
                goals_array.goals.push_back(goals);
            }
            ++search;
        }

        if ( finished == robots_status.size() )
        {
            //reset();
        }
        else
        {
            pub_robot_goals.publish(goals_array);
        }
    }

}


void OrderManager::subscribe_robot_odom()
{
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);

    for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
        const ros::master::TopicInfo& info = *it;
        if(std::regex_match(info.name, std::regex("/robot_(\\d)+/odom")))
        {
            subscribers.push_back(
                nodeHandle->subscribe(info.name, 1, &OrderManager::odomCallback, this));
        }
    }
}

void OrderManager::odomCallback(const nav_msgs::Odometry& odom) {
    std::string robot_name = odom.header.frame_id.substr(1, odom.header.frame_id.find_last_of('/')-1);

    float x = odom.pose.pose.position.x;
    float y = odom.pose.pose.position.y;
    float z = odom.pose.pose.position.z;

    tuw::ros_msgs::Pose *new_pose = new tuw::ros_msgs::Pose(x, y, z, 0, 0, 0);

    std::map<std::string, tuw::ros_msgs::Pose*>::iterator search = subscribed_robots.find(robot_name);

    if ( search == subscribed_robots.end() )
        return;


    search->second = new_pose;


    std::map<std::string, int>::iterator attached_good = attached_goods.find(robot_name);
    if ( attached_good != attached_goods.end() )
    {
        int good_id = attached_good->second;
        if (good_id >= 0)
        {
            publishGoodPosition(good_id, odom.pose.pose);

        }
    }
}


tuw_multi_robot_msgs::Good *OrderManager::findGoodByRobotName(std::string robot_name)
{
    return findGoodByGoodId(findGoodIdByRobotName(robot_name));
}

tuw_multi_robot_msgs::Good *OrderManager::findGoodByGoodId(int id)
{
    if ( id < 0 )
        return nullptr;
    for ( int i=0; i<goods.size(); ++i )
    {
        tuw_multi_robot_msgs::Good *good = &(goods.at(i));
        if ( good->good_id == id )
            return good;
    }
    return nullptr;
}

int OrderManager::findGoodIdByRobotName(std::string robot_name)
{
    int good_id;
    for( auto const& pair : transport_pairs )
    {
        if (robot_name == pair.robot_name )
        {
            return pair.good_id;
        }
    }
    return -1;
}

void OrderManager::publishGoodPosition(int good_id, geometry_msgs::Pose pose)
{
    tuw_multi_robot_msgs::GoodPosition goodPosition;
    goodPosition.good_id = good_id;
    goodPosition.position = pose;
    pub_good_pose.publish(goodPosition);
}

void OrderManager::publishPickup(std::string robot_name, int good_id)
{
    tuw_multi_robot_msgs::Pickup pickup;
    pickup.robot_name = robot_name;
    pickup.good_id = good_id;
    pub_pickup.publish(pickup);
}

} // end namespace tuw_order_manager

int main(int argc, char** argv)
{

    tuw_order_manager::OrderManager *controller = new tuw_order_manager::OrderManager(argc, argv);
    controller->run();

    return 0;
}
