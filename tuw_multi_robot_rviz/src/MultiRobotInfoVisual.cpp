#include "tuw_multi_robot_rviz/MultiRobotInfoVisual.h"

namespace tuw_multi_robot_rviz {
  MultiRobotInfoVisual::MultiRobotInfoVisual(Ogre::SceneManager* _scene_manager, Ogre::SceneNode* _parent_node) : scene_manager_(_scene_manager), frame_node_(_parent_node->createChildSceneNode())
  {}

  MultiRobotInfoVisual::~MultiRobotInfoVisual()
  {
    scene_manager_->destroySceneNode(frame_node_);
  }

  void MultiRobotInfoVisual::recycle()
  {
    std::vector<std::string> mark_for_deletion;
    for (auto &elem : recycle_map_)
    {
      if (elem.second >= recycle_thresh_)
      {
        mark_for_deletion.push_back(elem.first);
      } else {
        elem.second++;
      }
    }
    for (const std::string &elem : mark_for_deletion)
    {
      robot2pose_map_.erase(robot2pose_map_.find(elem));
      recycle_map_.erase(recycle_map_.find(elem));
    }
  }

  void MultiRobotInfoVisual::setMessage(const tuw_multi_robot_msgs::RobotInfo::ConstPtr &_msg)
  {
    map_iterator it = robot2pose_map_.find(_msg->robot_name);

    if (it == robot2pose_map_.end())
    {
      robot2pose_map_.insert(internal_map_type(_msg->robot_name, boost::circular_buffer<geometry_msgs::PoseWithCovariance>(default_size_)));
      it = robot2pose_map_.find(_msg->robot_name);
      recycle_map_.insert(std::pair<std::string, unsigned int>(_msg->robot_name, 0));
    }

    it->second.push_front(_msg->pose);
    recycle_map_.find(_msg->robot_name)->second = 0;
    recycle();

    robot_poses_.resize(robot2pose_map_.size());
    int i;
    for (i=0, it=robot2pose_map_.begin(); it!=robot2pose_map_.end(); ++i, ++it)
    {
      //TODO: for now only the last pose is shown
      if (!robot_poses_[i])
      {
        robot_poses_[i] = std::make_shared<rviz::Arrow>(this->scene_manager_, this->frame_node_, 1.5,0.2,0.2,0.25);
      }

      const auto pose = it->second[0].pose;
      Ogre::Vector3 position(pose.position.x, pose.position.y, pose.position.z);
      Ogre::Quaternion orientation(pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z);
      Ogre::Quaternion orient_x = Ogre::Quaternion( Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Y );
      orientation = orientation * orient_x;

      robot_poses_[i]->setPosition(position);
      robot_poses_[i]->setOrientation(orientation);

    }
  }

  void MultiRobotInfoVisual::setFramePosition(const Ogre::Vector3& position)
  {
    frame_node_->setPosition(position);
  }

  void MultiRobotInfoVisual::setFrameOrientation(const Ogre::Quaternion& orientation)
  {
    frame_node_->setOrientation(orientation);
  }

  void MultiRobotInfoVisual::setScalePose(float scale)
  {
    for (const auto &arrow : robot_poses_)
    {
      arrow->setScale(Ogre::Vector3(scale,scale,scale));
    }
  }

  void MultiRobotInfoVisual::setColorPose(Ogre::ColourValue color)
  {
    for (const auto &arrow : robot_poses_)
    {
      arrow->setColor(color);
    }
  }
}
