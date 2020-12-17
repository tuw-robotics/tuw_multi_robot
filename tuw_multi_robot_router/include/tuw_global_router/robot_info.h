/*
 * Copyright (c) 2017, <copyright holder> <email>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY <copyright holder> <email> ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <copyright holder> <email> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef TUW_MULTI_ROBOT_ROBOT_ROUTER_INFO_H
#define TUW_MULTI_ROBOT_ROBOT_ROUTER_INFO_H

//ROS
#include <ros/ros.h>
#include <tuw_multi_robot_msgs/RobotGoalsArray.h>
#include <tuw_multi_robot_msgs/RobotGoals.h>
#include <tuw_multi_robot_msgs/RobotInfo.h>
#include <nav_msgs/Odometry.h>
#include <tuw_multi_robot_msgs/Graph.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tuw_multi_robot_msgs/RouterStatus.h>
#include <dynamic_reconfigure/server.h>
#include <tuw_multi_robot_router/routerConfig.h>

#include <tuw_global_router/router.h>
#include <tuw_global_router/mrr_utils.h>
#include <opencv2/core/core.hpp>

//TODO disable got_map if not used

namespace multi_robot_router
{
class RobotInfo : public tuw_multi_robot_msgs::RobotInfo
{
public:
    RobotInfo ()
    : tuw_multi_robot_msgs::RobotInfo()
    , online_(Online::inactive)
    , activeTime_(1.0)
    {
    }
    RobotInfo (const tuw_multi_robot_msgs::RobotInfo& o)
    : tuw_multi_robot_msgs::RobotInfo(o)
    , online_(Online::inactive)
    , activeTime_(1.0)
    {
    }
    RobotInfo (const std::string &name)
    : tuw_multi_robot_msgs::RobotInfo()
    , online_(Online::inactive)
    , activeTime_(1.0)
    {
            robot_name = name;
    }
        enum class Online
        {
            inactive,
            active,
            fixed
        };
        
        void updateInfo(const tuw_multi_robot_msgs::RobotInfo &info);
        
        /**
         * crates subsribers and publisher using a given namespace
         * @param n node handler
         * @param robot_name_as_namespace on true it will use the robots name as namespace prefix
         **/
        void initTopics(ros::NodeHandle &n,  bool robot_name_as_namespace);
        
        Online getOnlineStatus() const;
        void updateOnlineStatus ( const float updateTime );
        
        /**
         * returns vehilce radius based on the robots shape
         * @ToDo a caching must be implemented for non circular shapes
         * @return radius
         **/
        float radius() const;
        
        Eigen::Vector3d getPose() const;
        /**
         * returns the indes of the robot with a name
         * @return index or data.size() if no matching element was found
         **/
        static size_t findIdx(const std::vector<std::shared_ptr<RobotInfo> > &data, const std::string &name);
        /**
         * returns a reference to the robot with a name
         * @return ref or data.end() if no matching element was found
         **/
        static std::vector<std::shared_ptr<RobotInfo> >::iterator findObj(std::vector<std::shared_ptr<RobotInfo> > &data, const std::string &name);
        
        bool compareName(const std::shared_ptr<RobotInfo> data) const;
        
        ros::Subscriber subOdom_;
        ros::Publisher pubPaths_;
        ros::Publisher pubRoute_;
        void callback_odom ( const nav_msgs::Odometry &msg);
    private:
        Online online_;
        float activeTime_;
};
typedef std::shared_ptr<RobotInfo> RobotInfoPtr;
typedef std::vector<std::shared_ptr<RobotInfo> >::iterator RobotInfoPtrIterator;

} // namespace multi_robot_router
#endif // TUW_MULTI_ROBOT_ROBOT_ROUTER_INFO_H
