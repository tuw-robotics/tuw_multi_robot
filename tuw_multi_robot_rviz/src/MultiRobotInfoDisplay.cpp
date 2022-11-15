/**
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

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <tf2_ros/transform_listener.h>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/frame_manager_iface.hpp>

#include <tuw_multi_robot_msgs/msg/robot_info.hpp>

#include <tuw_multi_robot_rviz/MultiRobotInfoDisplay.hpp>
#include <tuw_multi_robot_rviz/MultiRobotInfoVisual.hpp>

namespace tuw_multi_robot_rviz {

// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
MultiRobotInfoDisplay::MultiRobotInfoDisplay() : rviz_common::RosTopicDisplay<tuw_multi_robot_msgs::msg::RobotInfo>(){

    property_scale_pose_.reset(new rviz_common::properties::FloatProperty ( "Scale Pose", 0.4,
            "Scale of the pose's pose.",
            this, SLOT ( updateScalePose() ) ));
    property_scale_pose_->setMin ( 0 );
    property_scale_pose_->setMax ( 10 );

    property_color_pose_.reset(new rviz_common::properties::ColorProperty ( "Color Pose", QColor ( 204, 51, 0 ),
            "Color to draw the pose's pose.",
            this, SLOT ( updateColorPose())));

    keep_alive_.reset(new rviz_common::properties::IntProperty("Keep Alive (s)",
                                            5,
                                            "The amount of seconds in which a robot is still displayed after it does not publish anymore. This value should be greater or equal than 1.",
                                            this,
                                            SLOT(onKeepAliveChanged()),
                                            this));
    keep_alive_->setMin(1);
    keep_alive_->setMax(20);
    keep_measurements_.reset(new rviz_common::properties::IntProperty("Keep Measurements",
                                                   5,
                                                   "The number of pose measurements that should be kept for each robot. Make sure that it is greater than 0.",
                                                   this,
                                                   SLOT(onKeepMeasurementsChanged()),
                                                   this));
    keep_measurements_->setMin(1);
    keep_measurements_->setMax(10000);

    robot_bool_properties_.reset(new rviz_common::properties::Property("Robots",
                                                   QVariant(),
                                                   "A tree view of all the available robots",
                                                   this));
}

void MultiRobotInfoDisplay::callbackRobotInfo( RobotInfo::ConstSharedPtr &msg )
{
   /* if (!visual_) return;

    ros::Time tic = ros::Time::now();
    // Here we call the rviz::FrameManager to get the transform from the
    // fixed frame to the frame in the header of this Imu message.  If
    // it fails, we can't do anything else so we return.
    Ogre::Quaternion orientation(1.0,0.0,0.0,0.0);
    Ogre::Vector3 position(0,0,0);

    if ( !context_->getFrameManager()->getTransform ( msg->header.frame_id, msg->header.stamp, position, orientation ) ) {
        ROS_DEBUG ( "Error transforming from frame '%s' to frame '%s'",
                    msg->header.frame_id.c_str(), qPrintable ( fixed_frame_ ) );
        return;
    }

    // Now set or update the contents of the visual.
    visual_->setMessage ( msg );
    //visual_->setFramePosition ( position );
    //visual_->setFrameOrientation ( orientation );
    //visual_->setScalePose ( property_scale_pose_->getFloat() );
    //visual_->setColorPose ( property_color_pose_->getOgreColor() );

    //auto dur = ros::Time::now() - tic;
    auto it = bool_properties_.find(msg->robot_name);
    if (it == bool_properties_.end())
    {
      std::unique_ptr<rviz::BoolProperty> bp;
      bp.reset(new rviz::BoolProperty(QString(msg->robot_name.c_str()),
                                true,
                                QString("display this robot?"),
                                robot_bool_properties_.get(),
                                SLOT(updateBoolProperty()),
                                this));
      bool_properties_.insert(std::pair<std::string,std::unique_ptr<rviz::BoolProperty>>(msg->robot_name, std::move(bp)));
    }*/

}
// After the top-level rviz::Display::initialize() does its own setup,
// it calls the subclass's onInitialize() function.  This is where we
// instantiate all the workings of the class.  We make sure to also
// call our immediate super-class's onInitialize() function, since it
// does important stuff setting up the message filter.
//
//  Note that "MFDClass" is a typedef of
// ``MessageFilterDisplay<message type>``, to save typing that long
// templated class name every time you need to refer to the
// superclass.
void MultiRobotInfoDisplay::onInitialize() {
    RTDClass::onInitialize();
    createRawNode();
    visual_.reset ( new MultiRobotInfoVisual ( raw_node_, context_->getSceneManager(), scene_node_ ) );
}

MultiRobotInfoDisplay::~MultiRobotInfoDisplay() {
}

// Clear the visual by deleting its object.
void MultiRobotInfoDisplay::reset() {
    RTDClass::reset();
}

// Set the current scale for the visual's pose.
void MultiRobotInfoDisplay::updateScalePose() {
    float scale = property_scale_pose_->getFloat();
    visual_->setScalePose ( scale );
}

// Set the current color for the visual's pose.
void MultiRobotInfoDisplay::updateColorPose() {
    Ogre::ColourValue color = property_color_pose_->getOgreColor();
    visual_->setColorPose ( color );
}

void MultiRobotInfoDisplay::updateBoolProperty()
{
  if (!visual_) return;
  for (const auto &it : bool_properties_)
  {
    if(!it.second->getBool())
    {
      visual_->disableRobot(it.first);
    } else {
      visual_->enableRobot(it.first);
    }
  }
}

void MultiRobotInfoDisplay::onKeepAliveChanged()
{
  if (!visual_) return;
  visual_->resetDuration(rclcpp::Duration(keep_measurements_->getInt(),0));
}

void MultiRobotInfoDisplay::onKeepMeasurementsChanged()
{
  if (!visual_) return;
  visual_->resetKeepMeasurementsCount(keep_alive_->getInt());
}

// This is our callback to handle an incoming message.
void MultiRobotInfoDisplay::processMessage ( RobotInfo::ConstSharedPtr msg ) {

  if (!visual_) return;

    rclcpp::Time tic = clock_->now();
    // Here we call the rviz::FrameManager to get the transform from the
    // fixed frame to the frame in the header of this Imu message.  If
    // it fails, we can't do anything else so we return.
    Ogre::Quaternion orientation(1.0,0.0,0.0,0.0);
    Ogre::Vector3 position(0,0,0);

    if ( !context_->getFrameManager()->getTransform ( msg->header.frame_id, msg->header.stamp, position, orientation ) ) {
        /*RCLCPP_DEBUG_EXPRESSION(rclcpp::Logger(), "Error transforming from frame '%s' to frame '%s'",
                    msg->header.frame_id.c_str(), qPrintable ( fixed_frame_ ) );*/
        return;
    }

    // Now set or update the contents of the visual.
    visual_->setMessage ( msg );
    //visual_->setFramePosition ( position );
    //visual_->setFrameOrientation ( orientation );
    //visual_->setScalePose ( property_scale_pose_->getFloat() );
    //visual_->setColorPose ( property_color_pose_->getOgreColor() );

    auto dur = clock_->now() - tic;
    auto it = bool_properties_.find(msg->robot_name);
    if (it == bool_properties_.end())
    {
      std::unique_ptr<rviz_common::properties::BoolProperty> bp;
      bp.reset(new rviz_common::properties::BoolProperty(QString(msg->robot_name.c_str()),
                                true,
                                QString("display this robot?"),
                                robot_bool_properties_.get(),
                                SLOT(updateBoolProperty()),
                                this));
      bool_properties_.insert(std::pair<std::string,std::unique_ptr<rviz_common::properties::BoolProperty>>(msg->robot_name, std::move(bp)));
    }

}

void MultiRobotInfoDisplay::createRawNode()
{
  raw_node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
  //sub_robot_info_ = raw_node -> template create_subscription<tuw_multi_robot_msgs::msg::RobotGoalsArray>("goals", 0);
  clock_ = raw_node_->get_clock();
}

} // end namespace tuw_multi_robot_rviz

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS (tuw_multi_robot_rviz::MultiRobotInfoDisplay,rviz_common::Display )
