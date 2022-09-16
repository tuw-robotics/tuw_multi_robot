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

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <rviz_common/ros_topic_display.hpp>
#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/display.hpp>

#include <tuw_multi_robot_msgs/msg/graph.hpp>

#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/frame_manager_iface.hpp>


#include <tuw_multi_robot_rviz/VoronoiGraphDisplay.hpp>
#include <tuw_multi_robot_rviz/VoronoiGraphVisual.hpp>

namespace tuw_multi_robot_rviz
{


// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
VoronoiGraphDisplay::VoronoiGraphDisplay() : rviz_common::RosTopicDisplay<tuw_multi_robot_msgs::msg::Graph>()
{
    color_path_property_ = new rviz_common::properties::ColorProperty ( "Path Color", QColor ( 50, 51, 204 ),
            "Color to draw the path.",
            this, SLOT ( updatePathColor() ) );

        scale_point_property_ = new rviz_common::properties::FloatProperty ( "Path Points Scale", 0.1,
            "Scale of the path points.",
            this, SLOT ( updatePointScale() ) );
		scale_point_property_->setMin ( 0 );
		scale_point_property_->setMax ( 1 );
		
	
    history_length_property_ = new rviz_common::properties::IntProperty ( "History Length", 1,
            "Number of prior measurements to display.",
            this, SLOT ( updateHistoryLength() ) );
    history_length_property_->setMin ( 1 );
    history_length_property_->setMax ( 100000 );
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
void VoronoiGraphDisplay::onInitialize() {
    RTDClass::onInitialize();
    updateHistoryLength();
}

//VoronoiGraphDisplay::~VoronoiGraphDisplay() {}

// Clear the visuals by deleting their objects.
void VoronoiGraphDisplay::reset() {
    RTDClass::reset();
    //visuals_.clear();
}


// Set the current color values for each visual.
void VoronoiGraphDisplay::updatePathColor() {
    Ogre::ColourValue color = color_path_property_->getOgreColor();
    for ( auto& visualsI: visuals_ ) { visualsI->setPathColor ( color ); }
}

// Set the number of past visuals to show.
void VoronoiGraphDisplay::updateHistoryLength() {
    visuals_.rset_capacity ( history_length_property_->getInt() );
}

void VoronoiGraphDisplay::updatePointScale()
{
	float scale = scale_point_property_->getFloat();
	for ( auto& visualsI: visuals_ ) { visualsI->setPointScale ( scale ); }
}

void VoronoiGraphDisplay::updatePathScale()
{
	float scale = scale_path_property_->getFloat();
	for ( auto& visualsI: visuals_ ) { visualsI->setPathScale ( scale ); }
}

// This is our callback to handle an incoming message.
void VoronoiGraphDisplay::processMessage ( Graph::ConstSharedPtr msg ) {
    // Here we call the rviz::FrameManager to get the transform from the
    // fixed frame to the frame in the header of this Imu message.  If
    // it fails, we can't do anything else so we return.
    Ogre::Quaternion orientation;
    Ogre::Vector3 position;
    if ( !context_->getFrameManager()->getTransform ( msg->header.frame_id,
            msg->header.stamp,
            position, orientation ) ) {
        //ROS_DEBUG ( "Error transforming from frame '%s' to frame '%s'",
        //            msg->header.frame_id.c_str(), qPrintable ( fixed_frame_ ) );
        return;
    }
    
    // We are keeping a circular buffer of visual pointers.  This gets
    // the next one, or creates and stores it if the buffer is not full
    boost::shared_ptr<VoronoiGraphVisual> visual;
    if ( visuals_.full() ) {
        visual = visuals_.front();
    } else {
        visual.reset ( new VoronoiGraphVisual ( context_->getSceneManager(), scene_node_ ) );
    }

    // Now set or update the contents of the chosen visual.
    visual->setMessage          ( msg );
    visual->setFramePosition    ( position );
    visual->setFrameOrientation ( orientation );
    
    visual->setPathColor     ( color_path_property_->getOgreColor() );

    // And send it to the end of the circular buffer
    visuals_.push_back ( visual );
}


} // end namespace rviz_plugin_tutorials

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.hpp> 
PLUGINLIB_EXPORT_CLASS(tuw_multi_robot_rviz::VoronoiGraphDisplay, rviz_common::Display)