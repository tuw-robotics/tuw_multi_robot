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

    arrow_.reset(new rviz::Arrow(scene_manager_, NULL, 2.0f, 0.2f, 0.5f, 0.35f));
    arrow_->setColor(0.0f, 1.0f, 0.0f, 1.0f);
    arrow_->getSceneNode()->setVisible(false);

    state_ = state::Position;

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
    state_ = state::Position;

    maxRobots_ = nr_robots_->getInt();

    if (robotCount_ >= maxRobots_)
    {
        robotCount_ = 0;

        for (auto &fn : flag_nodes_)
        {
            fn->detachAllObjects();
            fn->removeAndDestroyAllChildren();
            group_robot_goals_->removeChildren(0, -1);
        }
        flag_nodes_.clear();
        flag_positions_.clear();
        arrows_robot2flag_.clear();
    }

    if (moving_flag_node_)
    {
        moving_flag_node_->setVisible(true);

        state_ = state::Position;
        current_flag_property_ = new rviz::VectorProperty("Goal " + QString::number(flag_nodes_.size()));
        current_flag_property_->setReadOnly(true);
        flag_angles_.clear();
        group_robot_goals_->addChild(current_flag_property_);
    }
}

void MultiRobotGoalSelector::deactivate()
{
    if (moving_flag_node_)
    {
        moving_flag_node_->setVisible(false);
        state_ = state::Position;
        delete current_flag_property_;
        current_flag_property_ = NULL;
        flag_angles_.clear();
    }
}

void MultiRobotGoalSelector::make_quaternion(geometry_msgs::Pose::_orientation_type &q, double pitch, double roll, double yaw)
{
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);

    q.w = cy * cr * cp + sy * sr * sp;
    q.x = cy * sr * cp - sy * cr * sp;
    q.y = cy * cr * sp + sy * sr * cp;
    q.z = sy * cr * cp - cy * sr * sp;
}

void MultiRobotGoalSelector::make_quaternion(Ogre::Quaternion &q, double pitch, double roll, double yaw)
{
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);

    q.w = cy * cr * cp + sy * sr * sp;
    q.x = cy * sr * cp - sy * cr * sp;
    q.y = cy * cr * sp + sy * sr * cp;
    q.z = sy * cr * cp - cy * sr * sp;
}

int MultiRobotGoalSelector::processMouseEvent(rviz::ViewportMouseEvent &event)
{
    if (!moving_flag_node_)
    {
        return Render;
    }

    Ogre::Vector3 intersection;
    Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);

     if (event.leftDown())
     {
            if (rviz::getPointOnPlaneFromWindowXY(event.viewport,
                                          ground_plane,
                                          event.x, event.y, intersection))
            {

              moving_flag_node_->setVisible(true);
              moving_flag_node_->setPosition(intersection);
              current_flag_property_->setVector(intersection);

              flag_positions_.push_back(intersection);

              arrow_->setPosition(intersection);
              arrow_->getSceneNode()->setVisible(true);

              state_ = state::Orientation;

            }

     } else if (event.type == QEvent::MouseMove && event.left()) {
            //compute angle in x-y plane

            Ogre::Vector3 cur_pos;
            Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );

            if( rviz::getPointOnPlaneFromWindowXY( event.viewport,
                                                   ground_plane,
                                                   event.x, event.y, cur_pos ))
            {
              double angle = atan2( cur_pos.y - arrow_->getPosition().y, cur_pos.x - arrow_->getPosition().x );

              //we need base_orient, since the arrow goes along the -z axis by default (for historical reasons)
              Ogre::Quaternion orient_x = Ogre::Quaternion( Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Y );

              arrow_->setOrientation( Ogre::Quaternion( Ogre::Radian(angle), Ogre::Vector3::UNIT_Z ) * orient_x );

              Ogre::Quaternion q_from_angle;
              make_quaternion(q_from_angle, 0,0, angle);
              moving_flag_node_->setOrientation(q_from_angle);

            }

     } else if (event.leftUp()) {

            if (state_ == state::Orientation)
            {
              //compute angle in x-y plane
              Ogre::Vector3 cur_pos;
              Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );
              if(rviz::getPointOnPlaneFromWindowXY( event.viewport,
                                                    ground_plane,
                                                    event.x, event.y, cur_pos ))
              {
                 double angle = atan2( cur_pos.y - arrow_->getPosition().y, cur_pos.x - arrow_->getPosition().x );

                 Ogre::Quaternion q_from_angle;
                 make_quaternion(q_from_angle, 0,0, angle);
                 makeFlag(current_flag_property_->getVector(), q_from_angle);
                 //current_flag_property_ = new rviz::VectorProperty("Goal " + QString::number(flag_nodes_.size()));

                 //cool stuff here, since unique_ptr only allows 1 ptr count arrow_robot2flag_ will be automatically nullptr after this call
                 arrows_robot2flag_.push_back(std::move(arrow_robot2flag_));

                 flag_angles_.push_back(angle);

                 maxRobots_ = nr_robots_->getInt();
                 robotCount_++;

                 arrow_->getSceneNode()->setVisible(false);

                 if (robotCount_ >= maxRobots_) {

                   tuw_multi_robot_msgs::RobotGoalsArray array;
                   for (int i = 0; i < maxRobots_; i++)
                   {
                     Ogre::Vector3 position = flag_positions_[i];
                     tuw_multi_robot_msgs::RobotGoals goals;

                     geometry_msgs::Pose pose;
                     pose.position.x = position.x;
                     pose.position.y = position.y;
                     pose.position.z = 0.0;
                     make_quaternion(pose.orientation, 0, 0, flag_angles_[i]);
                     goals.destinations.push_back(pose);
                     goals.robot_name = robot_names_[i]->getStdString();

                     array.robots.push_back(goals);
                   }

                   array.header.stamp = ros::Time::now();
                   array.header.frame_id = context_->getFixedFrame().toStdString();
                   pubGoals_.publish(array);

                   return Render | Finished;
                }
            } // end get point

         } // end state == Orientation

    } // end if leftUp()
    else //if mouse is not clicked just update the moving flag
    {
       if (rviz::getPointOnPlaneFromWindowXY(event.viewport,
                                                 ground_plane,
                                                 event.x, event.y, intersection))
       {
          moving_flag_node_->setVisible(true);
          moving_flag_node_->setPosition(intersection);
          current_flag_property_->setVector(intersection);
          double length = Ogre::Vector3(0,0,0).distance(intersection);

          if (!arrow_robot2flag_)
          {

            arrow_robot2flag_.reset(new rviz::Arrow(scene_manager_, NULL, length - 0.5f, 0.2f, 0.5f, 0.35f));
            arrow_robot2flag_->setColor(1.0f, 0.0f, 0.0f, 1.0f);
            //TODO: replace position (0,0,0) with correct robot position and set visibility to true
            arrow_robot2flag_->getSceneNode()->setVisible(false);
            arrow_robot2flag_->setPosition(Ogre::Vector3(0,0,0));

          }

          arrow_robot2flag_->set(length - 0.5f, 0.2f, 0.5f, 0.35f);
          double angle = atan2( intersection.y - arrow_robot2flag_->getPosition().y, intersection.x - arrow_robot2flag_->getPosition().x );
          Ogre::Quaternion orient_x = Ogre::Quaternion( Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Y );
          arrow_robot2flag_->setOrientation( Ogre::Quaternion( Ogre::Radian(angle), Ogre::Vector3::UNIT_Z ) * orient_x );

       } else {
          moving_flag_node_->setVisible(false);
       }
    }

    return Render;
}

void MultiRobotGoalSelector::makeFlag(const Ogre::Vector3 &position, const Ogre::Quaternion &orientation)
{
    Ogre::SceneNode *node = scene_manager_->getRootSceneNode()->createChildSceneNode();
    Ogre::Entity *entity = scene_manager_->createEntity(flag_resource_);
    node->attachObject(entity);
    node->setVisible(true);
    node->setPosition(position);
    node->setOrientation(orientation);
    TextVisual *tw = new TextVisual(scene_manager_, node, Ogre::Vector3(0,
                                                                        0,
                                                                        node->getAttachedObject(0)->getBoundingBox().getSize().z + 0.15));
    tw->setCaption(std::to_string(flag_nodes_.size()));
    tw->setCharacterHeight(0.25);
    flag_nodes_.push_back(node);

}

} // namespace tuw_multi_robot_rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(tuw_multi_robot_rviz::MultiRobotGoalSelector, rviz::Tool)
