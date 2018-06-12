/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>

#include <ros/console.h>

#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/mesh_loader.h>
#include <rviz/geometry.h>

#include <tuw_multi_robot_rviz/MultiRobotGoalSelector.h>
#include <tuw_multi_robot_msgs/RobotGoals.h>
#include <tuw_multi_robot_msgs/RobotGoalsArray.h>

#include <string>

namespace tuw_multi_robot_rviz
{

MultiRobotGoalSelector::MultiRobotGoalSelector()
    : moving_flag_node_(NULL), current_flag_property_(NULL)
{
    shortcut_key_ = 'l';
    robotCount_ = 0;
    maxRobots_ = 3;

    pubGoals_ = nh_.advertise<tuw_multi_robot_msgs::RobotGoalsArray>("goals", 0);
}

MultiRobotGoalSelector::~MultiRobotGoalSelector()
{
    for (unsigned i = 0; i < flag_nodes_.size(); i++)
    {
        scene_manager_->destroySceneNode(flag_nodes_[i]);
    }
}

void MultiRobotGoalSelector::onInitialize()
{
    flag_resource_ = "package://tuw_multi_robot_rviz/media/flag.dae";

    if (rviz::loadMeshFromResource(flag_resource_).isNull())
    {
        ROS_ERROR("MultiRobotGoalSelector: failed to load model resource '%s'.", flag_resource_.c_str());
        return;
    }

    moving_flag_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
    Ogre::Entity *entity = scene_manager_->createEntity(flag_resource_);
    moving_flag_node_->attachObject(entity);
    moving_flag_node_->setVisible(false);

    group_robot_names_ = new rviz::Property("Robot Names");
    group_robot_goals_ = new rviz::Property("Robot Goals");
    nr_robots_ = new rviz::IntProperty("No. robtos", 3, "the nr of robots used for planning", nullptr, SLOT(onRobotNrChanged()), this);
    nr_robots_->setMin(0);

    getPropertyContainer()->addChild(nr_robots_);
    getPropertyContainer()->addChild(group_robot_names_);
    getPropertyContainer()->addChild(group_robot_goals_);

    currentRobotNr_ = 0;
    onRobotNrChanged();
}

void MultiRobotGoalSelector::onRobotNrChanged()
{
    if (currentRobotNr_ < nr_robots_->getInt())
    {
        for (uint32_t i = 0; i < nr_robots_->getInt() - currentRobotNr_; i++)
        {
            if (robot_names_.size() <= currentRobotNr_ + i)
            {
                robot_names_.push_back(new rviz::StringProperty("Robot " + QString::number(robot_names_.size()), "robot_" + QString::number(robot_names_.size())));
                group_robot_names_->addChild(robot_names_[currentRobotNr_ + i]);
            }
            else
            {
                robot_names_[currentRobotNr_ + i]->setHidden(false);
            }
        }
    }
    else if (currentRobotNr_ > nr_robots_->getInt())
    {
        for (uint32_t i = 0; i < currentRobotNr_ - nr_robots_->getInt(); i++)
        {
            robot_names_[nr_robots_->getInt() + i]->setHidden(true);
        }
    }

    currentRobotNr_ = nr_robots_->getInt();
}

void MultiRobotGoalSelector::activate()
{
    maxRobots_ = nr_robots_->getInt();

    if (robotCount_ >= maxRobots_)
    {
        robotCount_ = 0;

        for (auto &fn : flag_nodes_)
        {
            fn->detachAllObjects();
            group_robot_goals_->removeChildren(0, -1);
        }
        flag_nodes_.clear();
        vector_properties_.clear();
    }

    if (moving_flag_node_)
    {
        moving_flag_node_->setVisible(true);

        current_flag_property_ = new rviz::VectorProperty("Goal " + QString::number(flag_nodes_.size()));
        current_flag_property_->setReadOnly(true);
        group_robot_goals_->addChild(current_flag_property_);
    }
}

void MultiRobotGoalSelector::deactivate()
{
    if (moving_flag_node_)
    {
        moving_flag_node_->setVisible(false);
        delete current_flag_property_;
        current_flag_property_ = NULL;
    }
}

int MultiRobotGoalSelector::processMouseEvent(rviz::ViewportMouseEvent &event)
{
    if (!moving_flag_node_)
    {
        return Render;
    }

    Ogre::Vector3 intersection;
    Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);

    if (rviz::getPointOnPlaneFromWindowXY(event.viewport,
                                          ground_plane,
                                          event.x, event.y, intersection))
    {
        moving_flag_node_->setVisible(true);
        moving_flag_node_->setPosition(intersection);
        current_flag_property_->setVector(intersection);

        if (event.leftDown())
        {

            makeFlag(intersection);
            vector_properties_.push_back(current_flag_property_);
            current_flag_property_ = NULL;

            maxRobots_ = nr_robots_->getInt();
            robotCount_++;

            if (robotCount_ >= maxRobots_)
            {
                tuw_multi_robot_msgs::RobotGoalsArray array;
                for (int i = 0; i < maxRobots_; i++)
                {
                    Ogre::Vector3 position = vector_properties_[i]->getVector();
                    tuw_multi_robot_msgs::RobotGoals goals;

                    geometry_msgs::Pose pose;
                    pose.position.x = position.x;
                    pose.position.y = position.y;
                    pose.orientation.x = 0;
                    pose.orientation.y = 0;
                    pose.orientation.z = 0;
                    pose.orientation.w = 1;

                    goals.path_points.push_back(pose);
                    goals.robot_name = robot_names_[i]->getStdString();

                    array.goals.push_back(goals);
                }

                pubGoals_.publish(array);
            }

            return Render | Finished;
        }
    }
    else
    {
        moving_flag_node_->setVisible(false);
    }

    return Render;
}

void MultiRobotGoalSelector::makeFlag(const Ogre::Vector3 &position)
{
    Ogre::SceneNode *node = scene_manager_->getRootSceneNode()->createChildSceneNode();
    Ogre::Entity *entity = scene_manager_->createEntity(flag_resource_);
    node->attachObject(entity);
    node->setVisible(true);
    node->setPosition(position);
    flag_nodes_.push_back(node);
}

} // namespace tuw_multi_robot_rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(tuw_multi_robot_rviz::MultiRobotGoalSelector, rviz::Tool)