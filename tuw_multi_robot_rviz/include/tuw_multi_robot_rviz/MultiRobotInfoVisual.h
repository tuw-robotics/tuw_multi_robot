#ifndef MULTI_ROBOT_INFO_VISUAL_HPP
#define MULTI_ROBOT_INFO_VISUAL_HPP

#include <rviz/tool.h>
#include <ros/ros.h>
#include <rviz/properties/vector_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/ogre_helpers/arrow.h>
#include <OgreManualObject.h>
#include <rviz/ogre_helpers/shape.h>
#include <map>
#include <memory>
#include <vector>
#include <boost/circular_buffer.hpp>
#include "TextVisual.h"

#include <tuw_multi_robot_msgs/RobotInfo.h>
#include <tuw_multi_robot_msgs/Route.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace tuw_multi_robot_rviz
{

class RobotAttributes {

  public:

    using buf_type = boost::circular_buffer<geometry_msgs::PoseWithCovariance>;
    RobotAttributes(size_t id,
                    std::string &rname,
                    double rad,
                    buf_type &pose,
                    Ogre::ColourValue color,
                    Ogre::SceneManager *_scene_manager,
                    Ogre::SceneNode *_parent_node);

    ~RobotAttributes();

    std::string robot_name;
    double robot_radius;
    size_t robot_id;
    double path_length_;
    bool disabled;
    bool path_stale;

    std::vector<std::unique_ptr<TextVisual>> route_visuals;

    /** Color value set in rviz */
    void updateColor(Ogre::ColourValue &c)
    {
      color = c;
    }

    /** Disables the renderings of the robot poses are still stored */
    void setDisabled()
    {
      disabled = true;
      delete circle;
      delete arrow;
      circle = NULL;
      arrow = NULL;
    }

    void updatePose(const geometry_msgs::PoseWithCovariance &pose)
    {
      ros_poses.push_front(pose);
      current_pos = Ogre::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
      current_orient = Ogre::Quaternion(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
    }

    void updatePose(const buf_type &pose)
    {
      ros_poses = pose;

      if (pose.empty())
      {
        return;
      }

      const auto current_pose = pose.front();
      current_pos = Ogre::Vector3(current_pose.pose.position.x,
                                  current_pose.pose.position.y,
                                  current_pose.pose.position.z);

      current_orient = Ogre::Quaternion(current_pose.pose.orientation.w,
                                        current_pose.pose.orientation.x,
                                        current_pose.pose.orientation.y,
                                        current_pose.pose.orientation.z);
    }

    void setPoseBufferLength(size_t len)
    {
      ros_poses.resize(len);
    }

    /** get the path length, recomputed if necessary */
    double getPathLength()
    {
      if (path_stale)
      {
        updatePathLength();
      }
      return path_length_;
    }

    void render();

  private:
    buf_type ros_poses;
    std::shared_ptr<tuw_multi_robot_msgs::Route> route;
    ros::Subscriber sub_route;
    std::size_t seg_id_current;

    /** all things ogre */
    Ogre::ColourValue color;
    Ogre::Vector3 current_pos;
    Ogre::Quaternion current_orient;
    Ogre::SceneManager *scene_manager;
    Ogre::SceneNode *frame_node;
    Ogre::ManualObject* circle;
    Ogre::ManualObject* arrow;
    std::unique_ptr<TextVisual> text;

    /** callback for obtaining route information */
    void cbRoute(const ros::MessageEvent<const tuw_multi_robot_msgs::Route> &_event, int _topic);

    /** Update Circle rendering */
    void updateCircle(bool first_time=false);

    /** Update Arrow rendering */
    void updateArrow(bool first_time=false);

    /** Update the text rendering */
    void updateText(bool first_time=false);

    /** Make complete robot rendering */
    void make_robot();

    /** Make complete route rendering */
    void make_route();

    /** Update old path information */
    void updatePathLength()
    {
      if (route == nullptr)
        return;

      double min_dist = std::numeric_limits<double>::max();

      //Assumes sorted path!!
      for (size_t i=seg_id_current; i < route->segments.size(); ++i)
      {
        const auto s = route->segments[i];
        const auto position_s = s.start.position;
        const auto position_e = s.end.position;
        Eigen::Vector3f ps(position_s.x, position_s.y, position_s.z);
        Eigen::Vector3f dvec (ps - Eigen::Vector3f(current_pos.x, current_pos.y, current_pos.z));
        double dist = dvec.norm();
        if (dist < min_dist)
        {
          min_dist = dist;
          seg_id_current = i;
        }
      }

      double acc_len = 0;
      for (size_t i=seg_id_current; i < route->segments.size(); ++i)
      {
        const auto s = route->segments[i];
        auto position_s = s.start.position;
        if (i==seg_id_current)
        {
          position_s.x = current_pos.x;
          position_s.y = current_pos.y;
          position_s.z = current_pos.z;
        }
        const auto position_e = s.end.position;
        const double dist = pow(position_e.x - position_s.x, 2) + pow(position_e.y - position_s.y,2) + pow(position_e.z - position_s.z,2);
        acc_len += sqrt(dist);
      }
      path_length_ = acc_len;
    }
  };

class MultiRobotInfoVisual
{

public:

  MultiRobotInfoVisual(Ogre::SceneManager *_scene_manager, Ogre::SceneNode *_parent_node);

  virtual ~MultiRobotInfoVisual();

  void setMessage(const tuw_multi_robot_msgs::RobotInfoConstPtr _msg);

  void setFramePosition( const Ogre::Vector3& _position);
  void setFrameOrientation( const Ogre::Quaternion& orientation );

  // Set the scale of the visual, which is an user-editable
  // parameter and therefore don't come from the RobotGoalsArrayStamped message.
  void setScalePose( float scale );

  // Set the color of the visual's Pose, which is an user-editable
  // parameter and therefore don't come from the RobotGoalsArrayStamped message.
  void setColorPose( Ogre::ColourValue color );

  void disableRobot( const std::string &rName );

  void enableRobot( const std::string &rName );

  void resetDuration( const ros::Duration &ts);

  void resetKeepMeasurementsCount ( const unsigned int c );

  void doRender();


private:
  using internal_map_type = std::pair<std::string, std::shared_ptr<RobotAttributes>>;
  using map_type = std::map<std::string, std::shared_ptr<RobotAttributes>>;
  using map_iterator = std::map<std::string, std::shared_ptr<RobotAttributes>>::iterator;
  using recycle_map_type = std::map<std::string, ros::Time>;

  std::vector<std::string> recycle();

  // The object implementing the actual pose shape
  //std::map<std::string, std::vector<Ogre::ManualObject*>> robot_renderings_map_;
  /** includes render info such as distance to travel and preconditions */
  //std::map<std::string, std::vector<std::unique_ptr<TextVisual>>> robot_route_renderings_map_;
  std::set<std::string> disabled_robots_;

  map_type robot2attribute_map_;
  recycle_map_type recycle_map_;

  int default_size_ = {5};
  int id_cnt = 0;
  ros::Duration recycle_thresh_ = ros::Duration(5,0);
  ros::Duration render_dur_thresh_ = ros::Duration(0.1);
  ros::Time last_render_time_;
  // A SceneNode whose pose is set to match the coordinate frame of
  // the Imu message header.
  Ogre::SceneNode* frame_node_;

  // The SceneManager, kept here only so the destructor can ask it to
  // destroy the ``frame_node_``.
  Ogre::SceneManager* scene_manager_;

  // The pose Shape object's scale
  float scale_pose_;

  // The pose Shape object's color
  Ogre::ColourValue color_pose_ = Ogre::ColourValue(1,0,0,1);

  // The variance Shape object's color
  Ogre::ColourValue color_variance_;

};

}

#endif
