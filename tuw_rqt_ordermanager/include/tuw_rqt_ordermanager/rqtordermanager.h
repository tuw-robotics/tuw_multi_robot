#ifndef TUW_RQT_ORDER_MANAGER_H
#define TUW_RQT_ORDER_MANAGER_H

#include <rqt_gui_cpp/plugin.h>
#include <tuw_rqt_ordermanager/graphicsview.h>
#include <tuw_rqt_ordermanager/ui_rqtordermanager.h>

#include <tuw_rqt_ordermanager/robotdialog.h>
#include <tuw_rqt_ordermanager/stationdialog.h>
#include <tuw_rqt_ordermanager/orderdialog.h>

#include <tuw_multi_robot_srvs/StationManagerStationProtocol.h>
#include <tuw_multi_robot_srvs/StationManagerControlProtocol.h>

#include <tuw_rqt_ordermanager/item_robot.h>
#include <tuw_rqt_ordermanager/item_order.h>
#include <tuw_rqt_ordermanager/item_station.h>
#include <tuw_rqt_ordermanager/map_transformation.h>
#include <QWidget>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <tuw_multi_robot_msgs/OrderArray.h>
#include <tuw_multi_robot_msgs/OrderPosition.h>
#include <tuw_multi_robot_msgs/RobotInfo.h>
#include <tuw_multi_robot_msgs/Station.h>
#include <tuw_multi_robot_msgs/StationArray.h>

#include <mutex>


namespace tuw_rqt_ordermanager
{

class RQTOrdermanager : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  RQTOrdermanager();
  virtual void initPlugin(qt_gui_cpp::PluginContext&);
  virtual void shutdownPlugin();
  virtual void saveSettings(
      qt_gui_cpp::Settings& plugin_settings, 
      qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(
      const qt_gui_cpp::Settings& plugin_settings,
      const qt_gui_cpp::Settings& instance_settings);

  // Comment in to signal that the plugin has a way to configure it
  // bool hasConfiguration() const;
  // void triggerConfiguration();

  void mapCallback(const nav_msgs::OccupancyGrid&);
  void robotInfoCallback(const tuw_multi_robot_msgs::RobotInfo&);
  void orderPositionCallback(const tuw_multi_robot_msgs::OrderPosition&);
  void stationsCallback(const tuw_multi_robot_msgs::StationArray&);

private:
  MapTransformation map_transformation_;

  Ui::RQTOrdermanagerWidget ui_;
  QWidget* widget_;
  QGraphicsScene scene_;
  std::vector<ros::Subscriber> subscriptions_;
  ros::Publisher pub_orders_;

  float map_height_;
  float map_origin_position_x_;
  float map_origin_position_y_;
  float map_origin_position_z_;
  float map_resolution_;

  ItemStation* findStationByName(std::string);
  int findUnusedStationId();
  std::string findUnusedStationName();

  std::vector<QColor> order_colors_;
  std::mutex* mtx_lst_stations;

public slots:
  void setMap(const nav_msgs::OccupancyGrid&);
  void robotInfoHandle(const tuw_multi_robot_msgs::RobotInfo&);
  void orderPositionHandle(const tuw_multi_robot_msgs::OrderPosition&);
  void stationsHandle(const tuw_multi_robot_msgs::StationArray&);

  void newRobot();
  void deleteRobot();
  void editRobot();
  void setDrawBoundingRects(bool);

  void newStation(float x=0, float y=0, float z=0);
  void ordersItemSelectionChanged();
  void deleteStation();
  void deleteStationByName(std::string);
  void lockingDeleteStationByName(std::string);
  void editStation();

  void newOrder();
  void deleteOrder();
  void editOrder();
  void sendOrders();

  void orderAddStation(std::string);
  void orderClearPoses();
  void requestUpdateOnce();

signals:
  void mapChanged(const nav_msgs::OccupancyGrid);
  void robotInfoReceived(const tuw_multi_robot_msgs::RobotInfo&);
  void orderPositionReceived(const tuw_multi_robot_msgs::OrderPosition&);
  void stationsReceived(const tuw_multi_robot_msgs::StationArray&);

};

}  // namespace tuw_rqt_ordermanager
#endif  // TUW_RQT_ORDER_MANAGER_H
