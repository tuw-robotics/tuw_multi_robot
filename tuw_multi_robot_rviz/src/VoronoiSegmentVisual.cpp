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

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/ogre_helpers/line.h>
#include <tuw_multi_robot_msgs/Vertex.h>
#include <tuw_multi_robot_rviz/VoronoiSegmentVisual.h>

namespace tuw_multi_robot_rviz
{

VoronoiSegmentVisual::VoronoiSegmentVisual(Ogre::SceneManager *scene_manager, Ogre::SceneNode *parent_node)
{
	scene_manager_ = scene_manager;

	// Ogre::SceneNode s form a tree, with each node storing the
	// transform (position and orientation) of itself relative to its
	// parent.  Ogre does the math of combining those transforms when it
	// is time to render.
	//
	// Here we create a node to store the pose of the MarkerDetection's header frame
	// relative to the RViz fixed frame.
	frame_node_ = parent_node->createChildSceneNode();

	// initialize global variables
	colorPath_ = Ogre::ColourValue(255, 0, 0);
	scalePoint_ = 0.1;
	scalePath_ = 1;
}

VoronoiSegmentVisual::~VoronoiSegmentVisual()
{
	// Destroy the frame node since we don't need it anymore.
	scene_manager_->destroySceneNode(frame_node_);
}

void VoronoiSegmentVisual::setMessage(const tuw_multi_robot_msgs::Graph::ConstPtr &msg)
{
	static double timeOld_;
	if (timeOld_ == msg->header.stamp.toSec())
	{
		return;
	}
	timeOld_ = msg->header.stamp.toSec();

	pathLine.resize(msg->vertices.size() * 4);
	for (size_t i = 0; i < msg->vertices.size(); ++i)
	{
		tuw_multi_robot_msgs::Vertex seg = msg->vertices[i];
		geometry_msgs::Point p1 = seg.path.front();
		geometry_msgs::Point p2 = seg.path.back();

		// 	Ogre::Quaternion rotation  = Ogre::Quaternion ( Ogre::Radian( (*spline_)(i / (double)pointsNrPath_ )(2) + atan2(v_y, v_x) ), Ogre::Vector3::UNIT_Z );
		Line l = {
			(float)((p1.x + 0.5)  + msg->origin.position.x),
			(float)((p1.y + 0.5)  + msg->origin.position.y),
			(float)((p2.x + 0.5)  + msg->origin.position.x),
			(float)((p2.y + 0.5)  + msg->origin.position.y)};
		float z1 = p1.z  + msg->origin.position.z;
		float z2 = p2.z  + msg->origin.position.z;

		Line l1 = offsetLine(l,  (0.5 + seg.width / 2));
		Line l2 = offsetLine(l,  (-0.5 - seg.width / 2));

		pathLine[i * 4].reset(new rviz::Line(scene_manager_, frame_node_));
		pathLine[i * 4]->setColor(colorPath_);
		pathLine[i * 4]->setPoints(Ogre::Vector3(l1.x0, l1.y0, z1), Ogre::Vector3(l1.x1, l1.y1, z2));
		pathLine[i * 4]->setScale(Ogre::Vector3(scalePath_, scalePath_, scalePath_));

		pathLine[i * 4 + 1].reset(new rviz::Line(scene_manager_, frame_node_));
		pathLine[i * 4 + 1]->setColor(colorPath_);
		pathLine[i * 4 + 1]->setPoints(Ogre::Vector3(l2.x0, l2.y0, z1), Ogre::Vector3(l2.x1, l2.y1, z2));
		pathLine[i * 4 + 1]->setScale(Ogre::Vector3(scalePath_, scalePath_, scalePath_));

		pathLine[i * 4 + 2].reset(new rviz::Line(scene_manager_, frame_node_));
		pathLine[i * 4 + 2]->setColor(colorPath_);
		pathLine[i * 4 + 2]->setPoints(Ogre::Vector3(l1.x0, l1.y0, z1), Ogre::Vector3(l2.x0, l2.y0, z2));
		pathLine[i * 4 + 2]->setScale(Ogre::Vector3(scalePath_, scalePath_, scalePath_));

		pathLine[i * 4 + 3].reset(new rviz::Line(scene_manager_, frame_node_));
		pathLine[i * 4 + 3]->setColor(colorPath_);
		pathLine[i * 4 + 3]->setPoints(Ogre::Vector3(l2.x1, l2.y1, z1), Ogre::Vector3(l1.x1, l1.y1, z2));
		pathLine[i * 4 + 3]->setScale(Ogre::Vector3(scalePath_, scalePath_, scalePath_));
	}
}

VoronoiSegmentVisual::Line VoronoiSegmentVisual::offsetLine(VoronoiSegmentVisual::Line _l, float d)
{
	Line ret;
	float dx = _l.x0 - _l.x1;
	float dy = _l.y0 - _l.y1;
	float l = std::sqrt(dx * dx + dy * dy);

	ret.x0 = _l.x0 + d * (-dy) / l;
	ret.x1 = _l.x1 + d * (-dy) / l;
	ret.y0 = _l.y0 + d * dx / l;
	ret.y1 = _l.y1 + d * dx / l;

	return ret;
}

// Position is passed through to the SceneNode.
void VoronoiSegmentVisual::setFramePosition(const Ogre::Vector3 &position)
{
	frame_node_->setPosition(position);
}

// Orientation is passed through to the SceneNode.
void VoronoiSegmentVisual::setFrameOrientation(const Ogre::Quaternion &orientation)
{
	frame_node_->setOrientation(orientation);
}

// Color is passed through to the Shape object.
void VoronoiSegmentVisual::setPathColor(Ogre::ColourValue color)
{
	colorPath_ = color;
	for (auto &pathXYi : pathLine)
	{
		pathXYi->setColor(colorPath_);
	}
}

void VoronoiSegmentVisual::setPathScale(float scale)
{
	scalePath_ = scale;
	for (auto &pathThetai : pathLine)
	{
		pathThetai->setScale(Ogre::Vector3(scalePath_, scalePath_, scalePath_));
	}
}

} // namespace tuw_multi_robot_rviz