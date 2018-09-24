
/**
  * @author Felix Koenig (felix.koenig@protonmail.com)
 */
#ifndef MULTI_ROBOT_INFO_DISPLAY_H
#define MULTI_ROBOT_INFO_DISPLAY_H

#ifndef Q_MOC_RUN
#include <rviz/message_filter_display.h>
#endif

#include <rviz/properties/bool_property.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <memory>
#include <tuw_multi_robot_msgs/RobotInfo.h>

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class ColorProperty;
class FloatProperty;
class IntProperty;
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
class MultiRobotInfoDisplay: public rviz::MessageFilterDisplay<tuw_multi_robot_msgs::RobotInfo>
{
Q_OBJECT
public:
  // Constructor.  pluginlib::ClassLoader creates instances by calling
  // the default constructor, so make sure you have one.
  MultiRobotInfoDisplay();
  virtual ~MultiRobotInfoDisplay();

  void callbackRobotInfo(const tuw_multi_robot_msgs::RobotInfoConstPtr &msg );
  // Overrides of protected virtual functions from Display.  As much
  // as possible, when Displays are not enabled, they should not be
  // subscribed to incoming data and should not show anything in the
  // 3D view.  These functions are where these connections are made
  // and broken.
protected:
  virtual void onInitialize();

  // A helper to clear this display back to the initial state.
  virtual void reset();


  // These Qt slots get connected to signals indicating changes in the user-editable properties.
private Q_SLOTS:
  void updateScalePose();
  void updateColorPose();
  void updateBoolProperty();
  void onKeepAliveChanged();
  void onKeepMeasurementsChanged();

  // Function to handle an incoming ROS message.
private:
  void processMessage(const tuw_multi_robot_msgs::RobotInfoConstPtr &msg );

  // Storage of the visual
  std::shared_ptr<MultiRobotInfoVisual> visual_ = nullptr;

  // User-editable property variables.
  std::shared_ptr<rviz::Property> robot_bool_properties_;
  std::shared_ptr<rviz::IntProperty> keep_measurements_;
  std::shared_ptr<rviz::IntProperty> keep_alive_;
  std::shared_ptr<rviz::FloatProperty> property_scale_pose_;
  std::shared_ptr<rviz::ColorProperty> property_color_pose_;
  std::map<std::string, std::shared_ptr<rviz::BoolProperty>> bool_properties_;
  ros::Subscriber sub_robot_info_;
  ros::NodeHandle nh_;
};

} // end namespace tuw_pose_rviz

#endif // POSE_WITH_COVARIANCE_DISPLAY_H
// %EndTag(FULL_SOURCE)%
