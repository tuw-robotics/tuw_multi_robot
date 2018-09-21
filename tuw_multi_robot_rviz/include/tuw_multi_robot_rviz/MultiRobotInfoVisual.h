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
#include <memory>
#include <vector>
#include <map>
#include <boost/circular_buffer.hpp>
#include "TextVisual.h"

#include <tuw_multi_robot_msgs/RobotInfo.h>
#include <geometry_msgs/PoseWithCovariance.h>

namespace tuw_multi_robot_rviz
{

class MultiRobotInfoVisual
{
public:
  struct RobotAttributes {
  public:
    boost::circular_buffer<geometry_msgs::PoseWithCovariance> pose;
    double robot_radius;
  };

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

  std::vector<Ogre::ManualObject*> make_robot(Ogre::Vector3 &position, Ogre::Quaternion &orientation, double rad);

private:
  using internal_map_type = std::pair<std::string,RobotAttributes>;
  using map_type = std::map<std::string,RobotAttributes>;
  using map_iterator = std::map<std::string,RobotAttributes>::iterator;
  using recycle_map_type = std::map<std::string, ros::Time>;

  std::vector<std::string> recycle();

  // The object implementing the actual pose shape
  std::map<std::string, std::vector<Ogre::ManualObject*>> robot_renderings_map_;
  std::set<std::string> disabled_robots_;

  map_type robot2pose_map_;
  recycle_map_type recycle_map_;

  int default_size_ = {5};
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

  void updateCircle(Ogre::ManualObject *, Ogre::Vector3 &, double rad, bool first_time=false);
  void updateArrow(Ogre::ManualObject *, Ogre::Vector3 &p, Ogre::Quaternion &q, double rad, bool first_time=false);
};

}

#endif
