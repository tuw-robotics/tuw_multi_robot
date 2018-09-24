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

#include <ros/ros.h>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreMatrix3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include "tuw_multi_robot_rviz/RobotGoalsArrayVisual.h"

namespace tuw_multi_robot_rviz {

RobotGoalsArrayVisual::RobotGoalsArrayVisual ( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node ) {
    scene_manager_ = scene_manager;

    // Ogre::SceneNode s form a tree, with each node storing the
    // transform (position and orientation) of itself relative to its
    // parent.  Ogre does the math of combining those transforms when it
    // is time to render.
    //
    // Here we create a node to store the pose of the MarkerDetection's header frame
    // relative to the RViz fixed frame.
    frame_node_ = parent_node->createChildSceneNode();

}

RobotGoalsArrayVisual::~RobotGoalsArrayVisual() {
    // Destroy the frame node since we don't need it anymore.
    scene_manager_->destroySceneNode ( frame_node_ );
}

void RobotGoalsArrayVisual::setMessage ( const tuw_multi_robot_msgs::RobotGoalsArray::ConstPtr& msg ) {
    
    goals_.resize(msg->robots.size());
    
    for (size_t i = 0; i < msg->robots.size(); i++){
        goals_[i].reset ( new rviz::Arrow( scene_manager_, frame_node_ ) );
        boost::shared_ptr<rviz::Arrow> arrow = goals_[i];
        /// @ToDo generate an error message
        if(msg->robots[i].destinations.size() == 0) {
            continue;  
        }
        
        /// @Info # if there are more than one points the first one is the start pose  else the current pose of the robot is used as start
        const geometry_msgs::Pose &pose = msg->robots[i].destinations.back();  
        
        Ogre::Vector3 position = Ogre::Vector3 ( pose.position.x, pose.position.y, pose.position.z );
        Ogre::Quaternion orientation = Ogre::Quaternion ( pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w );
        

        // Arrow points in -Z direction, so rotate the orientation before display.
        // TODO: is it safe to change Arrow to point in +X direction?
        Ogre::Quaternion rotation1 = Ogre::Quaternion ( Ogre::Degree( -90 ), Ogre::Vector3::UNIT_Y );
        Ogre::Quaternion rotation2 = Ogre::Quaternion ( Ogre::Degree( -180 ), Ogre::Vector3::UNIT_X );
        orientation = rotation2 * rotation1 * orientation;

        arrow->setPosition( position );
        arrow->setOrientation( orientation );
    }
}

// Position is passed through to the SceneNode.
void RobotGoalsArrayVisual::setFramePosition ( const Ogre::Vector3& position ) {
    frame_node_->setPosition ( position );
}

// Orientation is passed through to the SceneNode.
void RobotGoalsArrayVisual::setFrameOrientation ( const Ogre::Quaternion& orientation ) {
    frame_node_->setOrientation ( orientation );
}

// Scale is passed through to the pose Shape object.
void RobotGoalsArrayVisual::setScalePose ( float scale ) {
    
    for(boost::shared_ptr<rviz::Arrow>& goal: goals_){
        goal->setScale ( Ogre::Vector3 ( scale, scale, scale ));
    }
    scale_pose_ = scale;
}

// Color is passed through to the pose Shape object.
void RobotGoalsArrayVisual::setColorPose ( Ogre::ColourValue color ) {
    for(boost::shared_ptr<rviz::Arrow>& goal: goals_){
        goal->setColor ( color );;
    }
    color_pose_ = color;
}

} // end namespace tuw_multi_robot_rviz

