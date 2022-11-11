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

#ifndef POSE_WITH_COVARIANCE_DISPLAY_H
#define POSE_WITH_COVARIANCE_DISPLAY_H

#include <boost/shared_ptr.hpp>

#ifndef Q_MOC_RUN
#include <tuw_multi_robot_msgs/msg/robot_goals_array.hpp>
#include <rviz_common/ros_topic_display.hpp>
#endif

namespace Ogre
{
class SceneNode;
}

namespace rviz_common
{
namespace properties
{
class ColorProperty;
class FloatProperty;
}
}

// All the source in this plugin is in its own namespace.  This is not
// required but is good practice.
namespace tuw_multi_robot_rviz
{

class RobotGoalsArrayVisual;

// Here we declare our new subclass of rviz::Display.  Every display
// which can be listed in the "Displays" panel is a subclass of
// rviz::Display.
class RobotGoalsArrayDisplay: public rviz_common::RosTopicDisplay<tuw_multi_robot_msgs::msg::RobotGoalsArray>
{
Q_OBJECT
public:
  // Constructor.  pluginlib::ClassLoader creates instances by calling
  // the default constructor, so make sure you have one.
  RobotGoalsArrayDisplay();
  void onInitialize() override;
  void reset() override;
  virtual ~RobotGoalsArrayDisplay();

  // Overrides of protected virtual functions from Display.  As much
  // as possible, when Displays are not enabled, they should not be
  // subscribed to incoming data and should not show anything in the
  // 3D view.  These functions are where these connections are made
  // and broken.
//protected:
  //virtual void onInitialize();

  // A helper to clear this display back to the initial state.
  //virtual void reset();

  // These Qt slots get connected to signals indicating changes in the user-editable properties.
private Q_SLOTS:
  void updateScalePose();
  void updateColorPose();

  // Function to handle an incoming ROS message.
private:
  using GoalsArray = tuw_multi_robot_msgs::msg::RobotGoalsArray;
  void processMessage( GoalsArray::ConstSharedPtr msg ) override;

  // Storage of the visual
  boost::shared_ptr<RobotGoalsArrayVisual> visual_;

  // User-editable property variables.
  rviz_common::properties::FloatProperty* property_scale_pose_;
  rviz_common::properties::ColorProperty* property_color_pose_;
};

} // end namespace tuw_pose_rviz

#endif // POSE_WITH_COVARIANCE_DISPLAY_H
// %EndTag(FULL_SOURCE)%

