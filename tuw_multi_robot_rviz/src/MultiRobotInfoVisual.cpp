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
      it.second.pose.resize(default_size_);
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

  void MultiRobotInfoVisual::updateCircle(Ogre::ManualObject *circle, Ogre::Vector3 &position, double rad, bool first_time)
  {
    if (first_time)
      circle->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);
    else
      circle->beginUpdate(0);

    double accuracy = 20;
    double factor = 2.0 * M_PI / accuracy;

    double theta=0.0;
    unsigned int point_index = 0;
    for (; theta < 2.0*M_PI; theta += factor) {
      double s_theta = rad * sin(theta);
      double c_theta = rad * cos(theta);
      circle->position(Ogre::Vector3(c_theta,s_theta,0) + position);
      circle->colour(color_pose_);
      circle->index(point_index++);
    }

    circle->index(0);
    circle->end();
  }

  void MultiRobotInfoVisual::updateArrow(Ogre::ManualObject* arrow_ptr, Ogre::Vector3 &position, Ogre::Quaternion &orientation, double rad, bool first_time)
  {
    if (first_time)
      arrow_ptr->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);
    else
      arrow_ptr->beginUpdate(0);

    double accuracy = 3;
    double factor = 2.0 * M_PI / accuracy;

    double theta=0.0;
    unsigned int point_index = 0;
    for (; point_index < accuracy; theta += factor) {
      double s_theta = rad * sin(theta);
      double c_theta = rad * cos(theta);
      Ogre::Vector3 local_pose = Ogre::Vector3(c_theta, s_theta, 0);
      Ogre::Matrix3 r_orient;
      orientation.ToRotationMatrix(r_orient);
      local_pose = r_orient * local_pose;
      if (point_index==0)
      {
        theta +=0.5;
      } else if (point_index==1)
      {
        theta -=1.0;
      }
      arrow_ptr->position(local_pose + position);
      arrow_ptr->colour(color_pose_);
      arrow_ptr->index(point_index++);
    }
    arrow_ptr->triangle(0,1,2);

    arrow_ptr->index(0);
    arrow_ptr->end();
  }

  std::vector<Ogre::ManualObject*> MultiRobotInfoVisual::make_robot(Ogre::Vector3 &position, Ogre::Quaternion &orientation, double rad)
  {
    Ogre::ManualObject *arrow_ptr = scene_manager_->createManualObject("r_arrow_" + std::to_string(this->robot_renderings_map_.size()));
    Ogre::ManualObject *circle = scene_manager_->createManualObject("r_circle_" + std::to_string(this->robot_renderings_map_.size()));

    //Arrow
    {
      updateArrow(arrow_ptr, position, orientation, rad, true);

      frame_node_->createChildSceneNode()->attachObject(arrow_ptr);
    }

    //Circle
    {
      updateCircle(circle, position, rad, true);

      frame_node_->createChildSceneNode()->attachObject(circle);
    }

    return std::vector<Ogre::ManualObject*>{ arrow_ptr, circle };
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
      const auto pose = it->second.pose.front().pose;
      double robot_rad = it->second.robot_radius;
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
                                       std::vector<Ogre::ManualObject*>>(
                                       it->first,
                                       make_robot(position, orientation,robot_rad)));
      } else {
         if (it_arrows->second.size())
         {
           Ogre::ManualObject *r_obj = it_arrows->second[0];
           updateArrow(r_obj, position, orientation, robot_rad);

           Ogre::ManualObject* circle = it_arrows->second[1];
           updateCircle(circle, position, robot_rad);
           //r_obj->setPosition(position);
           //r_obj->setOrientation(orientation);
         }
      }
    } //end for

    last_render_time_ = ros::Time::now();
  }

  void MultiRobotInfoVisual::setMessage(const tuw_multi_robot_msgs::RobotInfoConstPtr _msg)
  {
    map_iterator it = robot2pose_map_.find(_msg->robot_name);

    if (it == robot2pose_map_.end())
    {
      RobotAttributes ra;
      ra.pose = boost::circular_buffer<geometry_msgs::PoseWithCovariance>(default_size_);
      ra.robot_radius = _msg->shape_variables.size() ? _msg->shape_variables[0] : 1.0;

      robot2pose_map_.insert(internal_map_type(_msg->robot_name, ra));
      it = robot2pose_map_.find(_msg->robot_name);
      recycle_map_.insert(std::pair<std::string, ros::Time>(_msg->robot_name, ros::Time(0)));
    }

    it->second.pose.push_front(_msg->pose);
    it->second.robot_radius = _msg->shape_variables.size() ? _msg->shape_variables[0] : 1.0;

    if ((ros::Time::now() - last_render_time_) > render_dur_thresh_)
    {
      doRender();
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
    //for (auto &it : robot_renderings_map_)
    //{
    //  //for (auto &renderings : it.second)
    //  //{
    //  //  renderings->setScale(Ogre::Vector3(scale,scale,scale));
    //  //}
    //}
  }

  void MultiRobotInfoVisual::setColorPose(Ogre::ColourValue color)
  {
    color_pose_ = color;
    //for (auto &it : robot_renderings_map_)
    //{
    //  for (auto &renderings : it.second)
    //  {
    //    renderings->setColor(color.r,color.g,color.b,color.a);
    //  }
    //}
  }
}
