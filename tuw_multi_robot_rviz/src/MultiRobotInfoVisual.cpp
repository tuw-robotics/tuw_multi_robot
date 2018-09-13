#include "tuw_multi_robot_rviz/MultiRobotInfoVisual.h"

namespace tuw_multi_robot_rviz {
  MultiRobotInfoVisual::MultiRobotInfoVisual(Ogre::SceneManager* _scene_manager, Ogre::SceneNode* _parent_node) : scene_manager_(_scene_manager), frame_node_(_parent_node->createChildSceneNode())
  {}

  MultiRobotInfoVisual::~MultiRobotInfoVisual()
  {
    scene_manager_->destroySceneNode(frame_node_);
  }

  void MultiRobotInfoVisual::resetDuration(const ros::Duration &newDur)
  {
    recycle_thresh_ = newDur;
  }

  void MultiRobotInfoVisual::resetKeepMeasurementsCount(const unsigned int c)
  {
    default_size_ = c;
    for (auto &it : robot2pose_map_)
    {
      it.second.resize(default_size_);
    }
  }

  std::vector<std::string> MultiRobotInfoVisual::recycle()
  {
    std::vector<std::string> mark_for_deletion;
    auto ts_now = ros::Time::now();
    for (auto &elem : recycle_map_)
    {
      auto dur = ts_now - elem.second;
      if (dur > recycle_thresh_)
      {
        mark_for_deletion.push_back(elem.first);
      }
    }

    for (const std::string &elem : mark_for_deletion)
    {
      auto it_pose = robot2pose_map_.find(elem);
      if (it_pose != robot2pose_map_.end())
      {
        robot2pose_map_.erase(it_pose);
      }

      auto it_arrows = robot_arrows_map_.find(elem);
      if (it_arrows != robot_arrows_map_.end())
      {
        robot_arrows_map_.erase(it_arrows);
      }

      auto it_rcle = recycle_map_.find(elem);
      if (it_rcle != recycle_map_.end())
      {
        recycle_map_.erase(it_rcle);
      }
    }

    return std::move(mark_for_deletion);
  }

  void MultiRobotInfoVisual::enableRobot(const std::string &rName)
  {
    auto it = disabled_robots_.find(rName);
    if (it != disabled_robots_.end())
    {
      disabled_robots_.erase(it);
    }
  }

  void MultiRobotInfoVisual::disableRobot(const std::string &rName)
  {
    disabled_robots_.insert(rName);
    auto it_arrow = robot_arrows_map_.find(rName);
    if (robot_arrows_map_.end() != it_arrow)
    {
      //No need for detach objects call since this is handled in the SceneNode destructor
      robot_arrows_map_.erase(it_arrow);
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
    recycle_map_.find(_msg->robot_name)->second = ros::Time::now();
    auto recycled_robots_ = recycle();

    //Just for testing, this should never be needed.
    //auto rName = _msg->robot_name;
    //auto it_r = std::find_if(recycled_robots_.begin(), recycled_robots_.end(),
    //                         [&rName](const std::string &n)
    //                            {
    //                              if (n==rName) {
    //                                return true;
    //                              }});

    //if (it_r != recycled_robots_.end())
    //{
    //  return;
    //}

    //Always store the current robot pose, even if the robot is not visualized currently therefore check below
    if (disabled_robots_.find(_msg->robot_name) != disabled_robots_.end())
    {
      return;
    }

    auto it_arrows = robot_arrows_map_.find(_msg->robot_name);
    if (it_arrows == robot_arrows_map_.end())
    {
        robot_arrows_map_.insert(std::pair<std::string, std::shared_ptr<rviz::Arrow>>(_msg->robot_name,
                                 std::make_shared<rviz::Arrow>(
                                      this->scene_manager_,
                                      this->frame_node_, 1.5,0.2,0.2,0.25)));
        it_arrows = robot_arrows_map_.find(_msg->robot_name);
    }

    //TODO: for now only the last pose is shown
    const auto pose = it->second[0].pose;
    Ogre::Vector3 position(pose.position.x, pose.position.y, pose.position.z);
    Ogre::Quaternion orientation(pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z);
    Ogre::Quaternion orient_x = Ogre::Quaternion(Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Y);
    orientation = orientation * orient_x;

    it_arrows->second->setPosition(position);
    it_arrows->second->setOrientation(orientation);
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
    for (auto &it : robot_arrows_map_)
    {
      it.second->setScale(Ogre::Vector3(scale,scale,scale));
    }
  }

  void MultiRobotInfoVisual::setColorPose(Ogre::ColourValue color)
  {
    for (auto &it : robot_arrows_map_)
    {
      it.second->setColor(color);
    }
  }
}
