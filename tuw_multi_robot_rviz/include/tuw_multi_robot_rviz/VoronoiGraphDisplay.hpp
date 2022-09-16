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

#ifndef TUW_MULIT_ROBOT_RVIZ_VORONOIGRAPHDISPLAY_H
#define TUW_MULIT_ROBOT_RVIZ_VORONOIGRAPHDISPLAY_H

#include <boost/circular_buffer.hpp>
#include <boost/shared_ptr.hpp>

#include <rviz_common/ros_topic_display.hpp>
#include <rviz_common/display.hpp>

#include <tuw_multi_robot_msgs/msg/graph.hpp>

#ifndef Q_MOC_RUN
#include <boost/circular_buffer.hpp>
#include <rviz_rendering/objects/arrow.hpp>
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
class ColorProperty;
class EnumProperty;
class FloatProperty;
class IntProperty;
}
}

// All the source in this plugin is in its own namespace.  This is not
// required but is good practice.
namespace tuw_multi_robot_rviz
{

class VoronoiGraphVisual;

// BEGIN_TUTORIAL
// Here we declare our new subclass of rviz::Display.  Every display
// which can be listed in the "Displays" panel is a subclass of
// rviz::Display.
//
// ImuDisplay will show a 3D arrow showing the direction and magnitude
// of the IMU acceleration vector.  The base of the arrow will be at
// the frame listed in the header of the Imu message, and the
// direction of the arrow will be relative to the orientation of that
// frame.  It will also optionally show a history of recent
// acceleration vectors, which will be stored in a circular buffer.
//
// The ImuDisplay class itself just implements the circular buffer,
// editable parameters, and Display subclass machinery.  The visuals
// themselves are represented by a separate class, ImuVisual.  The
// idiom for the visuals is that when the objects exist, they appear
// in the scene, and when they are deleted, they disappear.


//class VoronoiGraphDisplay: public rviz_common::RosTopicDisplay<tuw_multi_robot_msgs::msg::Graph>
class VoronoiGraphDisplay: public rviz_common::RosTopicDisplay<tuw_multi_robot_msgs::msg::Graph>
{
  Q_OBJECT
public:
  // Constructor.  pluginlib::ClassLoader creates instances by calling
  // the default constructor, so make sure you have one.
  VoronoiGraphDisplay();
  void onInitialize() override;
  void reset() override;
  //virtual ~VoronoiGraphDisplay();

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
  void updatePathColor();
  void updatePointScale();
  void updatePathScale();
  
  void updateHistoryLength();
  
  // Function to handle an incoming ROS message.*/
private:
  using Graph = tuw_multi_robot_msgs::msg::Graph;

  void processMessage( Graph::ConstSharedPtr msg ) override;

  // Storage for the list of visuals.  It is a circular buffer where
  // data gets popped from the front (oldest) and pushed to the back (newest)
  boost::circular_buffer<boost::shared_ptr<VoronoiGraphVisual> > visuals_;
  //std::shared_ptr<VoronoiGraphVisual> visuals_;
  
  // User-editable property variables.
  rviz_common::properties::ColorProperty* color_path_property_;
  rviz_common::properties::FloatProperty* scale_point_property_;
  rviz_common::properties::FloatProperty* scale_path_property_;
  rviz_common::properties::IntProperty* history_length_property_;
  
  enum LineStyle {
    LINES,
    BILLBOARDS
  };
};

} // end namespace tuw_multi_robot_rviz

#endif // TUW_MULIT_ROBOT_RVIZ_VORONOIGRAPHDISPLAY_H