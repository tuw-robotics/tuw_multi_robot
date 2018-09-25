#include "tuw_rqt_ordermanager/item_order.h"

#include <limits>

namespace tuw_rqt_ordermanager
{
ItemOrder::ItemOrder() : QObject(), QGraphicsItem(), QListWidgetItem()
{
  stations_ = new RWVector<std::string>();
  color_ = Qt::blue;
  colored_brush_ = new QBrush(color_);
  colored_pen_ = new QPen(color_);
  colored_pen_->setCosmetic(true);
  drawing_mode_ = DRAWING_MODE_PLAN;
  drawBoundingRect_ = false;
}

QRectF ItemOrder::boundingRect() const
{
  int minx = std::numeric_limits<int>::max();
  int miny = std::numeric_limits<int>::max();
  int maxx = std::numeric_limits<int>::min();
  int maxy = std::numeric_limits<int>::min();

  stations_->lock();
  if (stations_->size() == 0)
  {
    stations_->unlock();
    return QRectF(0, 0, 0, 0);
  }

  mtx_lst_stations_->lock();
  int found = 0;
  for (int i = 0; i < stations_->size(); ++i)
  {
    std::string _station_name = stations_->at(i);
    QList<QListWidgetItem*> matched_stations = lst_stations_->findItems(QString::fromStdString(_station_name), Qt::MatchExactly);
    if (matched_stations.size() == 0)
      continue;
    ++found;
    geometry_msgs::Pose pose =
      ((ItemStation*)(matched_stations.at(0)))->getPose();

    if (minx > pose.position.x)
      minx = pose.position.x;
    if (miny > pose.position.y)
      miny = pose.position.y;
    if (maxx < pose.position.x)
      maxx = pose.position.x;
    if (maxy < pose.position.y)
      maxy = pose.position.y;
  }

  mtx_lst_stations_->unlock();
  stations_->unlock();

  if (found == 0)
    return QRectF(0, 0, 0, 0);

  return QRectF(
      minx - ITEM_SIZE/2,
      miny - ITEM_SIZE/2,
      maxx - minx + ITEM_SIZE,
      maxy - miny + ITEM_SIZE);
}

/*
 * paint this orders route from station to station
 */
void ItemOrder::paint(
    QPainter* painter,
    const QStyleOptionGraphicsItem* option,
    QWidget* widget)
{
  setTransformOriginPoint(QPointF(0, 0));
  painter->setBrush(*colored_brush_);
  painter->setPen(*colored_pen_);

  QPointF* last_point = NULL;

  stations_->lock();
  mtx_lst_stations_->lock();
  for (int i = 0; i < stations_->size(); ++i)
  {
    std::string _station_name = stations_->at(i);
    QList<QListWidgetItem*> matched_stations = lst_stations_->findItems(QString::fromStdString(_station_name), Qt::MatchExactly);
    if (matched_stations.size() == 0)
      continue;
    geometry_msgs::Pose pose =
      ((ItemStation*)(matched_stations.at(0)))->getPose();

    if (last_point != NULL)
      painter->drawLine(
          QPointF(
            last_point->x(),
            last_point->y()
          ),
          QPointF(
            pose.position.x,
            pose.position.y
          ));

    last_point = new QPointF(pose.position.x, pose.position.y);
  }

  for (int i = 0; i < stations_->size(); ++i)
  {
    std::string _station_name = stations_->at(i);
    QList<QListWidgetItem*> matched_stations = lst_stations_->findItems(QString::fromStdString(_station_name), Qt::MatchExactly);
    if (matched_stations.size() == 0)
      continue;
    geometry_msgs::Pose pose =
      ((ItemStation*)(matched_stations.at(0)))->getPose();

    painter->setBrush(*colored_brush_);
    QRectF* rect = new QRectF(
        pose.position.x - ITEM_SIZE / 2,
        pose.position.y - ITEM_SIZE / 2,
        ITEM_SIZE,
        ITEM_SIZE);
    painter->drawRoundedRect(*rect, 0, 0);
    if (i == 0)
    {
      painter->setBrush(QBrush(QColor(0, 0, 0, 0)));
      painter->drawEllipse(QPointF(
            pose.position.x,
            pose.position.y
          ),
          ITEM_SIZE * 2,
          ITEM_SIZE * 2);
    }
  }

  mtx_lst_stations_->unlock();
  stations_->unlock();

  if (drawing_mode_ == DRAWING_MODE_EXEC && current_pose_ != NULL )
  {
    color_.setAlpha(255);
    QBrush* brush = new QBrush(color_);
    painter->setBrush(*brush);
    QRectF* rect = new QRectF(
        current_pose_->position.x - ITEM_SIZE / 2,
        current_pose_->position.y - ITEM_SIZE / 2,
        ITEM_SIZE,
        ITEM_SIZE);
    painter->drawRoundedRect(*rect, 0, 0);
  }

  // debug bounding rect:
  if (drawBoundingRect_)
  {
    painter->setBrush(*(new QColor(0, 255, 0, 0)));
    painter->drawRect(this->boundingRect());
  }
}

void ItemOrder::setOrderName(QString order_name)
{
  order_name_ = order_name;
  setText(order_name);
}

void ItemOrder::setId(int id)
{
  id_ = id;
}

int ItemOrder::getId()
{
  return id_;
}

QString ItemOrder::getOrderName()
{
  return order_name_;
}

void ItemOrder::addStation(std::string station_name)
{
  stations_->lock();
  stations_->push_back(station_name);
  stations_->unlock();
}

void ItemOrder::clearStations()
{
  stations_->lock();
  stations_->clear();
  stations_->unlock();
}

RWVector<std::string>* ItemOrder::getStations()
{
  return stations_;
}

void ItemOrder::setColor(QColor& color)
{
  color_ = color;
  colored_brush_ = new QBrush(color_);
  colored_pen_ = new QPen(color_);
  colored_pen_->setCosmetic(true);
}

void ItemOrder::setDrawingMode(int drawing_mode)
{
  drawing_mode_ = drawing_mode;

  if (drawing_mode_ == DRAWING_MODE_ACTIVE)
    color_.setAlpha(255);
  else
    color_.setAlpha(50);

  colored_brush_ = new QBrush(color_);
  colored_pen_ = new QPen(color_);
  colored_pen_->setCosmetic(true);
}

void ItemOrder::setCurrentPose(geometry_msgs::Pose* pose)
{
  current_pose_ = pose;
}

void ItemOrder::setStationsList(QListWidget* lst_stations)
{
  lst_stations_ = lst_stations;
}

void ItemOrder::setDrawBoundingRect(bool drawBoundingRect)
{
  drawBoundingRect_ = drawBoundingRect;
}
void ItemOrder::setLstStationsLock(std::mutex* mtx_lst_stations)
{
  mtx_lst_stations_ = mtx_lst_stations;
}

}  // end namespace tuw_rqt_ordermanager
