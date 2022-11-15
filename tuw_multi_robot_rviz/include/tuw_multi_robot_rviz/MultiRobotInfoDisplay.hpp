
/**
  * @author Felix Koenig (felix.koenig@protonmail.com)
 */
#ifndef MULTI_ROBOT_INFO_DISPLAY_H
#define MULTI_ROBOT_INFO_DISPLAY_H

#ifndef Q_MOC_RUN
#include <rviz_common/ros_topic_display.hpp>
#endif

//#include <rviz/properties/bool_property.h>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <memory>
#include <tuw_multi_robot_msgs/msg/robot_info.hpp>

#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>

namespace Ogre
{
class SceneNode;
}

namespace rviz_common
{
namespace properties
{
class ColorProperty;
class FloatProperty;
class IntProperty;
}
}

class QString;

// All the source in this plugin is in its own namespace.  This is not
// required but is good practice.
namespace tuw_multi_robot_rviz
{

class MultiRobotInfoVisual;

// Here we declare our new subclass of rviz::Display.  Every display
// which can be listed in the "Displays" panel is a subclass of
// rviz::Display.
class MultiRobotInfoDisplay: public rviz_common::RosTopicDisplay<tuw_multi_robot_msgs::msg::RobotInfo>
{
Q_OBJECT
public:
  using RobotInfo = tuw_multi_robot_msgs::msg::RobotInfo;

  // Constructor.  pluginlib::ClassLoader creates instances by calling
  // the default constructor, so make sure you have one.
  MultiRobotInfoDisplay();
  virtual ~MultiRobotInfoDisplay();

  void callbackRobotInfo( RobotInfo::ConstSharedPtr &msg );
  // Overrides of protected virtual functions from Display.  As much
  // as possible, when Displays are not enabled, they should not be
  // subscribed to incoming data and should not show anything in the
  // 3D view.  These functions are where these connections are made
  // and broken.
//protected:
   void onInitialize() override;

  // A helper to clear this display back to the initial state.
   void reset() override;


  // These Qt slots get connected to signals indicating changes in the user-editable properties.
private Q_SLOTS:
  void updateScalePose();
  void updateColorPose();
  void updateBoolProperty();
  void onKeepAliveChanged();
  void onKeepMeasurementsChanged();

  // Function to handle an incoming ROS message.
private:
  void processMessage( RobotInfo::ConstSharedPtr msg ) override;
  void createRawNode();

  // Storage of the visual
  std::shared_ptr<MultiRobotInfoVisual> visual_ = nullptr;

  //rclcpp::Subscriber<RobotInfo>::SharedPtr sub_robot_info_;
  rclcpp::Node::SharedPtr raw_node_;
  rclcpp::Clock::SharedPtr clock_;

  // User-editable property variables.
  std::shared_ptr<rviz_common::properties::Property> robot_bool_properties_;
  std::shared_ptr<rviz_common::properties::IntProperty> keep_measurements_;
  std::shared_ptr<rviz_common::properties::IntProperty> keep_alive_;
  std::shared_ptr<rviz_common::properties::FloatProperty> property_scale_pose_;
  std::shared_ptr<rviz_common::properties::ColorProperty> property_color_pose_;
  std::map<std::string, std::shared_ptr<rviz_common::properties::BoolProperty>> bool_properties_;
  
};

} // end namespace tuw_pose_rviz

#endif // POSE_WITH_COVARIANCE_DISPLAY_H
// %EndTag(FULL_SOURCE)%
