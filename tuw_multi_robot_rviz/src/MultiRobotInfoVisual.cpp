#include <tuw_multi_robot_rviz/MultiRobotInfoVisual.h>
#include <boost/format.hpp>

namespace tuw_multi_robot_rviz {

  using RA = RobotAttributes;

  RA::RobotAttributes(size_t id,
                      std::string &robot_name,
                      double rad,
                      buf_type &pose,
                      Ogre::ColourValue color,
                      Ogre::SceneManager *_scene_manager,
                      Ogre::SceneNode *_parent_node)
                                      : robot_id(id),
                                        robot_name(robot_name),
                                        scene_manager(_scene_manager),
                                        frame_node(_parent_node),
                                        robot_radius(rad),
                                        color(color),
                                        disabled(false),
                                        path_stale(true),
                                        seg_id_current(0)
  {
    updatePose(pose);
    text = nullptr;
    arrow = nullptr;
    circle = nullptr;
    route = nullptr;
    sub_route = ros::NodeHandle("").subscribe<tuw_multi_robot_msgs::Route>(robot_name + "/route", 1, boost::bind(&RobotAttributes::cbRoute, this, _1, id));
    path_length_ = 0;
  }

  RA::~RobotAttributes()
  {
    //delete circle;
    //delete arrow;
  }

  void RA::render()
  {
    //TODO: this needs to be done otherwise rviz crashes when all arrows are painted at position zero (which is a bad indicator for rviz...)
    if (current_pos.x == 0 && current_pos.y == 0 && current_pos.z == 0)
      return;

    if (arrow == nullptr || circle == nullptr)
    {
        make_robot();
        make_route();
    } else {
        updateArrow();
        updateCircle();
        updateText();
    }
  }

  //only called once! not consecutively
  void RA::cbRoute(const ros::MessageEvent<const tuw_multi_robot_msgs::Route> &_event, int _topic)
  {
    route.reset(new tuw_multi_robot_msgs::Route (*_event.getMessage() ) );
  }

  void RA::updateCircle(bool first_time)
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
      double s_theta = robot_radius * sin(theta);
      double c_theta = robot_radius * cos(theta);
      circle->position(Ogre::Vector3(c_theta,s_theta,0) + current_pos);
      circle->colour(color);
      circle->index(point_index++);
    }

    circle->index(0);
    circle->end();
  }

  void RA::updateArrow(bool first_time)
  {
    if (first_time)
      arrow->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);
    else
      arrow->beginUpdate(0);

    double accuracy = 3;
    double factor = 2.0 * M_PI / accuracy;

    double theta=0.0;
    unsigned int point_index = 0;
    for (; point_index < accuracy; theta += factor) {
      double s_theta = robot_radius * sin(theta);
      double c_theta = robot_radius * cos(theta);
      Ogre::Vector3 local_pose = Ogre::Vector3(c_theta, s_theta, 0);
      Ogre::Matrix3 r_orient;
      current_orient.ToRotationMatrix(r_orient);
      local_pose = r_orient * local_pose;
      if (point_index==0)
      {
        theta +=0.5;
      } else if (point_index==1)
      {
        theta -=1.0;
      }
      arrow->position(local_pose + current_pos);
      arrow->colour(color);
      arrow->index(point_index++);
    }
    arrow->triangle(0,1,2);

    arrow->index(0);
    arrow->end();
  }

  void RA::make_route()
  {
    if (text == nullptr)
    {
      updateText(true);
    }
    else
    {
      updateText(false);
    }
  }

  void RA::updateText(bool first_time)
  {

    if (first_time)
    {
      text.reset(new TextVisual(scene_manager, frame_node, current_pos));
      text->setCharacterHeight(0.2);
    }

    std::string capt = (boost::format("%d: %.2f") % this->robot_id % this->getPathLength()).str();
    text->setCaption(capt);
    text->setPosition(current_pos - Ogre::Vector3(0.25,0.25,0));

  }

  void RA::make_robot()
  {
    arrow = this->scene_manager->createManualObject("r_arrow_" + std::to_string(robot_id));
    circle = this->scene_manager->createManualObject("r_circle_" + std::to_string(robot_id));

    //Arrow
    {
      updateArrow(true);

      frame_node->createChildSceneNode()->attachObject(arrow);
    }

    //Circle
    {
      updateCircle(true);

      frame_node->createChildSceneNode()->attachObject(circle);
    }
  }

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
    for (auto &it : robot2attribute_map_)
    {
      it.second->setPoseBufferLength(default_size_);
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
      auto it_pose = robot2attribute_map_.find(elem);
      if (it_pose != robot2attribute_map_.end())
      {
        robot2attribute_map_.erase(it_pose);
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
    auto it = robot2attribute_map_.find(rName);
    if (robot2attribute_map_.end() != it)
    {
      //No need for detach objects call since this is handled in the SceneNode destructor
      it->second->setDisabled();
    }
  }

  void MultiRobotInfoVisual::doRender()
  {
    for (map_iterator it = robot2attribute_map_.begin(); it != robot2attribute_map_.end(); ++it)
    {

      if (disabled_robots_.find(it->first) != disabled_robots_.end())
      {
        return;
      }

      //TODO: for now only the last pose is shown
      it->second->render();

    } //end for

    last_render_time_ = ros::Time::now();
  }

  void MultiRobotInfoVisual::setMessage(const tuw_multi_robot_msgs::RobotInfoConstPtr _msg)
  {
    map_iterator it = robot2attribute_map_.find(_msg->robot_name);
    if (it == robot2attribute_map_.end())
    {
      double robot_radius = _msg->shape_variables.size() ? _msg->shape_variables[0] : 1.0;
      boost::circular_buffer<geometry_msgs::PoseWithCovariance> pose;
      pose.set_capacity(default_size_);
      std::string rn = _msg->robot_name;
      std::shared_ptr<RA> robot_attr = std::make_shared<RA>(id_cnt++,
                                                            rn,
                                                            robot_radius,
                                                            pose,
                                                            color_pose_,
                                                            scene_manager_,
                                                            frame_node_);

      robot2attribute_map_.insert(internal_map_type(rn, robot_attr));
      it = robot2attribute_map_.find(_msg->robot_name);

      recycle_map_.insert(std::pair<std::string, ros::Time>(_msg->robot_name, ros::Time(0)));
    }

    it->second->updatePose(_msg->pose);
    it->second->robot_radius = _msg->shape_variables.size() ? _msg->shape_variables[0] : 1.0;

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
    for (auto &it : robot2attribute_map_)
    {
      it.second->updateColor(color);
    }
  }
}
