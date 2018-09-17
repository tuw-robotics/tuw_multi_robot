#include "tuw_multi_robot_rviz/MultiRobotInfoVisual.h"

namespace tuw_multi_robot_rviz {
  MultiRobotInfoVisual::MultiRobotInfoVisual(Ogre::SceneManager* _scene_manager, Ogre::SceneNode* _parent_node) : scene_manager_(_scene_manager), frame_node_(_parent_node->createChildSceneNode())
  {
    last_render_time_ = ros::Time::now();
  }

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

      auto it_arrows = robot_renderings_map_.find(elem);
      if (it_arrows != robot_renderings_map_.end())
      {
        robot_renderings_map_.erase(it_arrows);
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
    auto it_arrow = robot_renderings_map_.find(rName);
    if (robot_renderings_map_.end() != it_arrow)
    {
      //No need for detach objects call since this is handled in the SceneNode destructor
      robot_renderings_map_.erase(it_arrow);
    }
  }

  std::vector<rviz::Object*> MultiRobotInfoVisual::make_robot(Ogre::Vector3 &position, Ogre::Quaternion &orientation)
  {
    rviz::Object* arrow_ptr;
    //std::shared_ptr<rviz::BillboardLine> billboard_ptr;

    //Arrow
    {
      arrow_ptr = new rviz::Arrow(
                                 this->scene_manager_,
                                 this->frame_node_, 1.5,0.2,0.2,0.25);
      arrow_ptr->setScale(Ogre::Vector3(1,1,1));
      arrow_ptr->setColor(1,0,0,1);
    }

    {
      //billboard_ptr = std::make_shared<rviz::BillboardLine>(this->scene_manager_,
      //                                                      this->frame_node_);
      //billboard_ptr->setLineWidth(5.0);
      //billboard_ptr->setMaxPointsPerLine(2);
      //Ogre::Vector3 offset = Ogre::Vector3(1,0.0,0);
      //double accuracy = 50;
      //double factor = 2.0 * M_PI / accuracy;

      //billboard_ptr->setNumLines(static_cast<int>(accuracy));
      //double theta=0.0;
      //for (; theta < 2.0*M_PI; theta += factor) {
      //  double s_theta = sin(theta);
      //  double c_theta = cos(theta);
      //  billboard_ptr->addPoint(Ogre::Vector3(c_theta, 0, s_theta));
      //  theta += factor;
      //  s_theta = sin(theta);
      //  c_theta = cos(theta);
      //  billboard_ptr->addPoint(Ogre::Vector3(c_theta, 0, s_theta));
      //  billboard_ptr->newLine();
      //}
    }

    //billboard_ptr->setPosition(position);

    Ogre::Quaternion orient_x = Ogre::Quaternion(Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Y);
    arrow_ptr->setPosition(position);
    arrow_ptr->setOrientation(orientation * orient_x);

    return std::vector<rviz::Object*>{ arrow_ptr /*, billboard_ptr */};
  }

  void MultiRobotInfoVisual::doRender()
  {
    for (map_iterator it = robot2pose_map_.begin(); it != robot2pose_map_.end(); ++it)
    {
      if (disabled_robots_.find(it->first) != disabled_robots_.end())
      {
        return;
      }

      //TODO: for now only the last pose is shown
      const auto pose = it->second.front().pose;
      //TODO: this needs to be done otherwise rviz crashes when all arrows are painted at position zero (which is a bad indicator for rviz...)
      if (pose.position.x == 0 && pose.position.y == 0 && pose.position.z == 0)
        continue;
      Ogre::Vector3 position(pose.position.x, pose.position.y, pose.position.z);
      Ogre::Quaternion orientation(pose.orientation.w,
                                   pose.orientation.x,
                                   pose.orientation.y,
                                   pose.orientation.z);

      auto it_arrows = robot_renderings_map_.find(it->first);
      if (it_arrows == robot_renderings_map_.end())
      {
          robot_renderings_map_.insert(std::pair<std::string,
                                       std::vector<rviz::Object*>>(
                                         it->first,
                                         make_robot(position, orientation)));
      } else {
          for (int ii = 0; ii < it_arrows->second.size(); ++ii)
          {
            rviz::Object* r_obj = it_arrows->second[ii];
            if (dynamic_cast<rviz::Arrow*>(r_obj))
            {
              Ogre::Quaternion orient_x = Ogre::Quaternion(Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Y);
              r_obj->setPosition(position);
              r_obj->setOrientation(orientation * orient_x);
            }
            else
            {
              r_obj->setPosition(position);
              r_obj->setOrientation(orientation);
            }
          }
      }
    }

    last_render_time_ = ros::Time::now();
  }

  void MultiRobotInfoVisual::setMessage(const tuw_multi_robot_msgs::RobotInfoConstPtr _msg)
  {
    map_iterator it = robot2pose_map_.find(_msg->robot_name);

    if (it == robot2pose_map_.end())
    {
      robot2pose_map_.insert(internal_map_type(_msg->robot_name, boost::circular_buffer<geometry_msgs::PoseWithCovariance>(default_size_)));
      it = robot2pose_map_.find(_msg->robot_name);
      recycle_map_.insert(std::pair<std::string, ros::Time>(_msg->robot_name, ros::Time(0)));
    }

    it->second.push_front(_msg->pose);
    if ((ros::Time::now() - last_render_time_) > render_dur_thresh_)
    {
      //std::cout << "Rendering now " << std::endl;
      doRender();
      //std::cout << "Rendering finished " << std::endl;
    }

    //recycle_map_.find(_msg->robot_name)->second = ros::Time::now();
    //auto recycled_robots_ = recycle();

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


    //ROS_INFO("Tracking: %d poses", robot2pose_map_.size());
    //ROS_INFO("Rendering %d robots", robot_renderings_map_.size());
    //ROS_INFO("Disabled robots %d", disabled_robots_.size());

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
    for (auto &it : robot_renderings_map_)
    {
      for (auto &renderings : it.second)
      {
        renderings->setScale(Ogre::Vector3(scale,scale,scale));
      }
    }
  }

  void MultiRobotInfoVisual::setColorPose(Ogre::ColourValue color)
  {
    for (auto &it : robot_renderings_map_)
    {
      for (auto &renderings : it.second)
      {
        renderings->setColor(color.r,color.g,color.b,color.a);
      }
    }
  }
}
