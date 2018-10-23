/*
*/

#include <iostream>

#include "tuw_rqt_ordermanager/rqtordermanager.h"
#include "tuw_rqt_ordermanager/graphicsview.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>

#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsItem>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <QMetaType>
#include <QPixmap>
#include <QtGlobal>

#include <typeinfo>
#include <algorithm>
#include <string>
//#include <regex>

namespace tuw_rqt_ordermanager
{
RQTOrdermanager::RQTOrdermanager() : rqt_gui_cpp::Plugin(), widget_(0)
{
  setObjectName("TUW RQT Ordermanager");

  order_colors_.push_back("blue");
  order_colors_.push_back("red");
  order_colors_.push_back("green");
  order_colors_.push_back("orange");
  order_colors_.push_back("brown");
  order_colors_.push_back("yellow");
  order_colors_.push_back("pink");
  order_colors_.push_back("gold");
}

void RQTOrdermanager::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);

  mtx_lst_stations = new std::mutex();

  // add widget to the user interface
  context.addWidget(widget_);

  qRegisterMetaType<nav_msgs::OccupancyGrid>("nav_msgs::OccupancyGrid");
  qRegisterMetaType<tuw_multi_robot_msgs::RobotInfo>("tuw_multi_robot_msgs::RobotInfo");
  qRegisterMetaType<tuw_multi_robot_msgs::OrderPosition>("tuw_multi_robot_msgs::OrderPosition");
  qRegisterMetaType<tuw_multi_robot_msgs::StationArray>("tuw_multi_robot_msgs::StationArray");
  qRegisterMetaType<std::string>("std::string");

  connect(this, &RQTOrdermanager::mapChanged, this, &RQTOrdermanager::setMap, Qt::QueuedConnection);
  connect(this, &RQTOrdermanager::robotInfoReceived, this, &RQTOrdermanager::robotInfoHandle, Qt::QueuedConnection);
  connect(this, &RQTOrdermanager::orderPositionReceived, this, &RQTOrdermanager::orderPositionHandle, Qt::QueuedConnection);
  connect(this, &RQTOrdermanager::stationsReceived, this, &RQTOrdermanager::stationsHandle, Qt::QueuedConnection);

  /*
  connect(ui_.btn_new_robot, SIGNAL(clicked()), this, SLOT(newRobot()), Qt::QueuedConnection);
  connect(ui_.btn_delete_robot, SIGNAL(clicked()), this, SLOT(deleteRobot()), Qt::QueuedConnection);
  connect(ui_.btn_edit_robot, SIGNAL(clicked()), this, SLOT(editRobot()), Qt::QueuedConnection);
  */

  connect(ui_.btn_new_station, SIGNAL(clicked()), this, SLOT(newStation()), Qt::QueuedConnection);
  connect(ui_.btn_delete_station, SIGNAL(clicked()), this, SLOT(deleteStation()), Qt::QueuedConnection);
  connect(ui_.btn_edit_station, SIGNAL(clicked()), this, SLOT(editStation()), Qt::QueuedConnection);

  connect(ui_.btn_new_order, SIGNAL(clicked()), this, SLOT(newOrder()), Qt::QueuedConnection);
  connect(ui_.btn_delete_order, SIGNAL(clicked()), this, SLOT(deleteOrder()), Qt::QueuedConnection);
  connect(ui_.btn_edit_order, SIGNAL(clicked()), this, SLOT(editOrder()), Qt::QueuedConnection);

  connect(ui_.btn_start, SIGNAL(clicked()), this, SLOT(sendOrders()), Qt::QueuedConnection);
  connect(ui_.btn_clear_order_poses, SIGNAL(clicked()), this, SLOT(orderClearPoses()), Qt::QueuedConnection);

  connect(ui_.map_view, &GraphicsView::orderAddStation, this, &RQTOrdermanager::orderAddStation, Qt::QueuedConnection);
  connect(ui_.map_view, &GraphicsView::newStation, this, &RQTOrdermanager::newStation, Qt::QueuedConnection);
  connect(ui_.map_view, &GraphicsView::removeStation, this, &RQTOrdermanager::lockingDeleteStationByName, Qt::QueuedConnection);

  connect(ui_.lst_orders, &QListWidget::itemSelectionChanged, this, &RQTOrdermanager::ordersItemSelectionChanged, Qt::QueuedConnection);

  connect(ui_.cb_showBoundingRects, SIGNAL(clicked(bool)), this, SLOT(setDrawBoundingRects(bool)), Qt::QueuedConnection);

  ui_.map_view->setMapTransformation(&map_transformation_);
  ui_.map_view->setScene(&scene_);

  pub_orders_ = getNodeHandle().advertise<tuw_multi_robot_msgs::OrderArray>("/orders", 1);

  subscriptions_.push_back(
      getNodeHandle().subscribe("/map", 0, &tuw_rqt_ordermanager::RQTOrdermanager::mapCallback, this));
  subscriptions_.push_back(
      getNodeHandle().subscribe("/robot_info", 10, &tuw_rqt_ordermanager::RQTOrdermanager::robotInfoCallback, this));
  subscriptions_.push_back(
      getNodeHandle().subscribe("/order_position", 10, &tuw_rqt_ordermanager::RQTOrdermanager::orderPositionCallback, this));
  subscriptions_.push_back(
      getNodeHandle().subscribe("/stations", 10, &tuw_rqt_ordermanager::RQTOrdermanager::stationsCallback, this));

}

void RQTOrdermanager::shutdownPlugin()
{
  getNodeHandle().shutdown();
}

void RQTOrdermanager::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  // instance_settings.setValue(k, v)
}

void RQTOrdermanager::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  // v = instance_settings.value(k)
}

/*bool hasConfiguration() const
{
    return true;
}

void triggerConfiguration()
{
    // Usually used to open a dialog to offer the user a set of configuration
}*/

void RQTOrdermanager::mapCallback(const nav_msgs::OccupancyGrid& msg)
{
  // use signal/slot to avoid concurrent scene updates
  emit mapChanged(msg);
}

// store received map in a pixmap
void RQTOrdermanager::setMap(const nav_msgs::OccupancyGrid& map)
{
  int width = map.info.width;
  int height = map.info.height;

  map_transformation_.setMapHeight(height);
  map_transformation_.setMapResolution(map.info.resolution);
  map_transformation_.setMapOriginPositionX(map.info.origin.position.x);
  map_transformation_.setMapOriginPositionY(map.info.origin.position.y);
  map_transformation_.setMapOriginPositionZ(map.info.origin.position.z);

  int col = 0;
  int row = height;

  QPixmap* pix = new QPixmap(width, height);
  QPainter* paint = new QPainter(pix);

  for (int i = 0; i < map.data.size(); ++i)
  {
    int val = map.data[i];
    // val domain:
    // -1: unknown
    // 0-100: occupancy

    if (val == -1)
    {
      paint->setPen(*(new QColor(255, 255, 255, 255)));
      paint->drawPoint(col, row);
    }
    else
    {
      int g = (100 - val) / 100 * 255;
      paint->setPen(*(new QColor(g, g, g, 255)));
      paint->drawPoint(col, row);
    }

    col += 1;
    if (col == width)
    {
      row -= 1;
      col = 0;
    }
  }

  scene_.addPixmap(*pix);

  // does not work in initPlugin, here it does (probably thread issue):
  this->requestUpdateOnce();
}

// show dialog to create order
void RQTOrdermanager::newOrder()
{
  OrderDialog* dialog = new OrderDialog();
  // TODO: make sure id is unique
  int id = ui_.lst_orders->count();
  std::stringstream sstm;
  sstm << "order_" << id;
  std::string order_name = sstm.str();
  dialog->setOrderName(QString::fromStdString(order_name));
  int ret = dialog->exec();
  if (ret == QDialog::Accepted)
  {
    ItemOrder* ir = new ItemOrder();
    ir->setDrawBoundingRect(ui_.cb_showBoundingRects->isChecked());
    ir->setLstStationsLock(mtx_lst_stations);
    ir->setStationsList(ui_.lst_stations);

    ir->setId(id);
    ir->setOrderName(dialog->getOrderName());
    ir->setColor(order_colors_[id % order_colors_.size()]);

    ir->setZValue(2);

    scene_.addItem(ir);
    ui_.lst_orders->addItem(ir);
    ui_.lst_orders->setCurrentItem(ir);
  }
}

// delete order
void RQTOrdermanager::deleteOrder()
{
  QList<QListWidgetItem*> list = ui_.lst_orders->selectedItems();
  for (int i = 0; i < list.size(); ++i)
  {
    ItemOrder* ir = (ItemOrder*)list[i];
    delete ir;
  }
}

// show edit order dialog
void RQTOrdermanager::editOrder()
{
  QList<QListWidgetItem*> list = ui_.lst_orders->selectedItems();
  if (list.size() < 1)
    return;
  ItemOrder* ir = (ItemOrder*)list[0];
  OrderDialog* dialog = new OrderDialog();
  dialog->setOrderName(ir->getOrderName());

  int ret = dialog->exec();
  if (ret == QDialog::Accepted)
  {
    ir->setOrderName(dialog->getOrderName());
    scene_.update();
  }
}

/*
void RQTOrdermanager::newRobot()
{
  RobotDialog* dialog = new RobotDialog();
  int ret = dialog->exec();
  if (ret == QDialog::Accepted)
  {
    ItemRobot* ir = new ItemRobot();
    ir->setDrawBoundingRect(ui_.cb_showBoundingRects->isChecked());
    ir->setRobotName(dialog->getRobotName());

    scene_.addItem(ir);
    ui_.lst_robots->addItem(ir);
    ui_.lst_robots->setCurrentItem(ir);
  }
}

void RQTOrdermanager::deleteRobot()
{
  QList<QListWidgetItem*> list = ui_.lst_robots->selectedItems();
  for (int i = 0; i < list.size(); ++i)
  {
    ItemRobot* ir = (ItemRobot*)list[i];
    delete ir;
  }
}

void RQTOrdermanager::editRobot()
{
  QList<QListWidgetItem*> list = ui_.lst_robots->selectedItems();
  if (list.size() < 1)
    return;
  ItemRobot* ir = (ItemRobot*)list[0];
  RobotDialog* dialog = new RobotDialog();
  dialog->setRobotName(ir->getRobotName());

  int ret = dialog->exec();
  if (ret == QDialog::Accepted)
  {
    ir->setRobotName(dialog->getRobotName());
    scene_.update();
  }
}
*/

// enable or disable drawing of bounding rects of all items
void RQTOrdermanager::setDrawBoundingRects(bool checked)
{
  mtx_lst_stations->lock();
  for (int i = 0; i < ui_.lst_stations->count(); ++i)
  {
    ItemStation* is = (ItemStation*)ui_.lst_stations->item(i);
    is->setDrawBoundingRect(checked);
  }
  mtx_lst_stations->unlock();

  for (int i = 0; i < ui_.lst_robots->count(); ++i)
  {
    ItemRobot* is = (ItemRobot*)ui_.lst_robots->item(i);
    is->setDrawBoundingRect(checked);
  }
  for (int i = 0; i < ui_.lst_orders->count(); ++i)
  {
    ItemOrder* is = (ItemOrder*)ui_.lst_orders->item(i);
    is->setDrawBoundingRect(checked);
  }
}

// return station which is identified by station_name, or null
ItemStation* RQTOrdermanager::findStationByName(std::string station_name)
{
  for (int i = 0; i < ui_.lst_stations->count(); ++i)
  {
    ItemStation* is = (ItemStation*)ui_.lst_stations->item(i);
    if ( is->getStationName().toStdString() == station_name )
    {
      return is;
    }
  }

  return NULL;
}

// find a station_name which is not yet in use
std::string RQTOrdermanager::findUnusedStationName()
{
  bool found = false;
  int id=0;
  std::string station_name;
  mtx_lst_stations->lock();
  while (!found)
  {
    std::stringstream sstm;
    sstm << "station_" << id;
    station_name = sstm.str();
    QString q_station_name = QString::fromStdString(station_name);
    QList<QListWidgetItem*> matched_stations = ui_.lst_stations->findItems(q_station_name, Qt::MatchExactly);
    if (matched_stations.size() == 0)
      break;
    ++id;
  }
  mtx_lst_stations->unlock();
  return station_name;
}

// delete station by station_name but acquire lock to do so
void RQTOrdermanager::lockingDeleteStationByName(std::string station_name)
{
  mtx_lst_stations->lock();
  deleteStationByName(station_name);
  mtx_lst_stations->unlock();
}

// delete station by station_name. lock yourself!
void RQTOrdermanager::deleteStationByName(std::string station_name)
{
  ItemStation* is = findStationByName(station_name);
  tuw_multi_robot_srvs::StationManagerStationProtocol delStation;
  tuw_multi_robot_msgs::Station station;
  station.id = is->getId();
  station.name = is->getStationName().toStdString();

  delStation.request.request = "remove";
  delStation.request.station = station;
  if (ros::service::call("station_manager_station_service", delStation))
  {
    //TODO show message
  }
  else
  {
    //TODO show error
  }
}

// delete station currently selected in station list
void RQTOrdermanager::deleteStation()
{
  //TODO: delete station from orders as well
  mtx_lst_stations->lock();
  QList<QListWidgetItem*> list = ui_.lst_stations->selectedItems();
  for (int i = 0; i < list.size(); ++i)
  {
    std::string station_name = ((ItemStation*)list.at(i))->getStationName().toStdString();
    deleteStationByName(station_name);
  }
  mtx_lst_stations->unlock();
}

// show new station dialog
void RQTOrdermanager::newStation(float x, float y, float z)
{
  StationDialog* dialog = new StationDialog();
  dialog->setPositionX(x);
  dialog->setPositionY(y);
  dialog->setPositionZ(z);

  std::string proposed_station_name = findUnusedStationName();
  int proposed_id = 0;
  dialog->setStationName(QString::fromStdString(proposed_station_name));
  dialog->setStationId(proposed_id);
  
  int ret = dialog->exec();
  if (ret == QDialog::Accepted)
  {
    tuw_multi_robot_srvs::StationManagerStationProtocol addStation;
    tuw_multi_robot_msgs::Station station;
    station.id = dialog->getStationId();
    station.name = dialog->getStationName().toStdString();

    geometry_msgs::Pose* pose = new geometry_msgs::Pose();
    pose->position.x = dialog->getPositionX();
    pose->position.y = dialog->getPositionY();
    pose->position.z = dialog->getPositionZ();
    station.pose = *pose;

    addStation.request.request = "append";
    addStation.request.station = station;
    if (ros::service::call("station_manager_station_service", addStation))
    {
      //TODO show message
    }
    else
    {
      //TODO show error
    }
  }
}

void RQTOrdermanager::orderPositionCallback(const tuw_multi_robot_msgs::OrderPosition& gp)
{
  emit orderPositionReceived(gp);
}

void RQTOrdermanager::stationsCallback(const tuw_multi_robot_msgs::StationArray& sa)
{
  emit stationsReceived(sa);
}

void RQTOrdermanager::robotInfoCallback(const tuw_multi_robot_msgs::RobotInfo& ri)
{
  emit robotInfoReceived(ri);
}

// set current position of an order
void RQTOrdermanager::orderPositionHandle(const tuw_multi_robot_msgs::OrderPosition& gp)
{
  for (int i = 0; i < ui_.lst_orders->count(); ++i)
  {
    ItemOrder* ig = (ItemOrder*)ui_.lst_orders->item(i);
    if (ig->getId() == gp.order_id)
    {
      float x = map_transformation_.transformMapToScene(MapTransformation::TRANSFORM_X, gp.position.position.x);
      float y = map_transformation_.transformMapToScene(MapTransformation::TRANSFORM_Y, gp.position.position.y);
      float z = map_transformation_.transformMapToScene(MapTransformation::TRANSFORM_Z, gp.position.position.z);
      geometry_msgs::Pose* pose = new geometry_msgs::Pose();
      pose->position.x = x;
      pose->position.y = y;
      pose->position.z = z;

      ig->setCurrentPose(pose);
      scene_.update();
      break;
    }
  }
}

// receive stations list
void RQTOrdermanager::stationsHandle(const tuw_multi_robot_msgs::StationArray& sa)
{
  mtx_lst_stations->lock();
  ui_.lst_stations->clear();
  for (int i = 0; i < sa.stations.size(); ++i)
  {
    tuw_multi_robot_msgs::Station station = sa.stations[i];

    // station manager ignores id
    //station.id = i;

    ItemStation* is = new ItemStation();
    is->setDrawBoundingRect(ui_.cb_showBoundingRects->isChecked());
    is->setId(station.id);
    is->setStationName(QString::fromStdString(station.name));
    is->setPose(map_transformation_.transformMapToScene(station.pose));

    scene_.addItem(is);
    connect(is, &ItemStation::setActiveStation, ui_.map_view, &GraphicsView::setActiveStation, Qt::QueuedConnection);
    ui_.lst_stations->addItem(is);
  }
  mtx_lst_stations->unlock();
}

// set current robot positions
void RQTOrdermanager::robotInfoHandle(const tuw_multi_robot_msgs::RobotInfo& ri)
{
  std::string robot_name = ri.robot_name;
  QString q_robot_name = QString::fromStdString(robot_name);

  QList<QListWidgetItem*> matched_robots = ui_.lst_robots->findItems(q_robot_name, Qt::MatchExactly);
  ItemRobot* ir;
  if (matched_robots.size() == 0)
  {
    ir = new ItemRobot();
    ir->setDrawBoundingRect(ui_.cb_showBoundingRects->isChecked());
    ir->setRobotName(q_robot_name);
    ir->setZValue(1);
    ir->setRobotRadius(map_transformation_.transformMapToScene(MapTransformation::TRANSFORM_SCALAR, ri.shape_variables[0]));

    scene_.addItem(ir);
    ui_.lst_robots->addItem(ir);
  }
  else
  {
    ir = (ItemRobot*)matched_robots[0];
  }

  ir->setPose(map_transformation_.transformMapToScene(ri.pose.pose));

  scene_.update();
}

// publish /orders message
void RQTOrdermanager::sendOrders()
{
  tuw_multi_robot_msgs::OrderArray order_array_msg;
  for (int i = 0; i < ui_.lst_orders->count(); ++i)
  {
    ItemOrder* ir = (ItemOrder*)ui_.lst_orders->item(i);
    ir->setDrawingMode(DRAWING_MODE_EXEC);
    RWVector<std::string> * station_names = ir->getStations();
    station_names->lock();

    // stations empty, no need to transport it anywhere
    if (station_names->size() <= 1)
    {
      station_names->unlock();
      continue;
    }

    tuw_multi_robot_msgs::Order order_msg;

    order_msg.order_name = ir->getOrderName().toStdString();
    order_msg.order_id = ir->getId();

    // target stations:
    mtx_lst_stations->lock();
    for (int j = 0; j < station_names->size(); ++j)
    {
      std::string _station_name = station_names->at(j);
      ItemStation* station = findStationByName(_station_name);
      if ( station == nullptr )
        continue;

      tuw_multi_robot_msgs::Station msg_station;
      msg_station.name = station->getStationName().toStdString();
      msg_station.pose = map_transformation_.transformSceneToMap(
          station->getPose());
      msg_station.id = station->getId();
      order_msg.stations.push_back(msg_station);
    }
    mtx_lst_stations->unlock();
    station_names->unlock();

    order_array_msg.orders.push_back(order_msg);
  }

  pub_orders_.publish(order_array_msg);
}

// add station to selected order
void RQTOrdermanager::orderAddStation(std::string station_name)
{
  QList<QListWidgetItem*> list = ui_.lst_orders->selectedItems();
  if (list.size() < 1)
  {
    return;
  }
  ItemOrder* ir = (ItemOrder*)list[0];

  ir->addStation(station_name);
  scene_.update();
}

// remove all stations from an order
void RQTOrdermanager::orderClearPoses()
{
  QList<QListWidgetItem*> list = ui_.lst_orders->selectedItems();
  if (list.size() < 1)
  {
    return;
  }
  ItemOrder* ir = (ItemOrder*)list[0];
  ir->clearStations();
  scene_.update();
}

// request to stationmanager to publish stations
void RQTOrdermanager::requestUpdateOnce()
{
  tuw_multi_robot_srvs::StationManagerControlProtocol command;
  command.request.request = "update";
  command.request.addition = "once";
  if(ros::service::call("station_manager_control_service", command))
  {
    //TODO show message
  }
  else
  {
    //TODO show error
  }
}

// called when selected order changes
void RQTOrdermanager::ordersItemSelectionChanged()
{
  for (int i = 0; i < ui_.lst_orders->count(); ++i)
  {
    ItemOrder* ir = (ItemOrder*)ui_.lst_orders->item(i);
    ir->setDrawingMode(DRAWING_MODE_PLAN);
  }

  QList<QListWidgetItem*> list = ui_.lst_orders->selectedItems();
  for (int i = 0; i < list.size(); ++i)
  {
    ItemOrder* ir = (ItemOrder*)list[i];
    ir->setDrawingMode(DRAWING_MODE_ACTIVE);
  }
}

}  // end namespace tuw_rqt_ordermanager
PLUGINLIB_EXPORT_CLASS(tuw_rqt_ordermanager::RQTOrdermanager, rqt_gui_cpp::Plugin)
