/*
 * Copyright (c) 2012, Willow Garage, Inc.
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
#ifndef MULTI_ROBOT_GOAL_SELECTOR_H
#define MULTI_ROBOT_GOAL_SELECTOR_H

#include <rviz_common/tool.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_rendering/viewport_projection_finder.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>

#include <rviz_common/properties/vector_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/string_property.hpp>

#include <tuw_multi_robot_msgs/msg/robot_goals.hpp>
#include <tuw_multi_robot_msgs/msg/robot_goals_array.hpp>


#include <memory>
#include "TextVisual.hpp"

#ifndef Q_MOC_RUN
#include <rviz_rendering/objects/arrow.hpp>
#endif

#include <geometry_msgs/msg/pose.hpp>

namespace Ogre
{
class SceneNode;
//class Vector3;
class Quaternion;
}

namespace rviz
{
class VectorProperty;
class IntProperty;
class FloatProperty;
class StringProperty;
class VisualizationManager;
class ViewportMouseEvent;
}

namespace tuw_multi_robot_rviz
{

// BEGIN_TUTORIAL
// Here we declare our new subclass of rviz::Tool.  Every tool
// which can be added to the tool bar is a subclass of
// rviz::Tool.
class MultiRobotGoalSelector: public rviz_common::Tool
{
Q_OBJECT
public:
  MultiRobotGoalSelector( );
  ~MultiRobotGoalSelector();

  virtual void onInitialize();

  virtual void activate();
  virtual void deactivate();

  virtual int processMouseEvent( rviz_common::ViewportMouseEvent& event );

protected Q_SLOTS:
  void onRobotNrChanged();

protected:
  std::shared_ptr<rviz_rendering::ViewportProjectionFinder> projection_finder_;

private:
  enum state {
    Position,
    Orientation
  };

  void make_quaternion(geometry_msgs::msg::Pose::_orientation_type &q, double pitch, double roll, double yaw);
  void make_quaternion(Ogre::Quaternion &q, double pitch, double roll, double yaw);
  void makeFlag( const Ogre::Vector3& position, const Ogre::Quaternion &orientation);
  void createRawNode();

  rclcpp::Publisher<tuw_multi_robot_msgs::msg::RobotGoalsArray>::SharedPtr pubGoals_;
  rclcpp::Clock::SharedPtr clock_;
  std::vector<Ogre::SceneNode*> flag_nodes_;
  Ogre::SceneNode* moving_flag_node_;
  std::string flag_resource_;
  rviz_common::properties::VectorProperty* current_flag_property_;
  rviz_common::properties::IntProperty* nr_robots_;
  std::vector<rviz_common::properties::StringProperty*> robot_names_;
  rviz_common::properties::Property *group_robot_names_;
  rviz_common::properties::Property *group_robot_goals_;
  std::unique_ptr<rviz_rendering::Arrow> arrow_;
  std::unique_ptr<rviz_rendering::Arrow> arrow_robot2flag_;
  state state_;
  std::vector<double> flag_angles_;
  std::vector<std::unique_ptr<rviz_rendering::Arrow>> arrows_robot2flag_;
  std::vector<Ogre::Vector3> flag_positions_;
  
  uint32_t currentRobotNr_;
  uint32_t maxRobots_; 
  uint32_t robotCount_;
};
// END_TUTORIAL

} // end namespace rviz_plugin_tutorials

#endif // PLANT_FLAG_TOOL_H
