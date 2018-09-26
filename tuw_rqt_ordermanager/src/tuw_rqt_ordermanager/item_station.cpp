#include "tuw_rqt_ordermanager/item_station.h"

namespace tuw_rqt_ordermanager
{
ItemStation::ItemStation() : QObject(), QGraphicsItem(), QListWidgetItem()
{
  radius_ = 5;
  is_hovered_ = false;
  drawBoundingRect_ = false;
  setAcceptHoverEvents(true);
}

QRectF ItemStation::boundingRect() const
{
  float x = pose_.position.x;
  float y = pose_.position.y;
  float z = pose_.position.z;

  float hoverBorder = radius_;

  return QRectF(
      x - hoverBorder - radius_, 
      y - hoverBorder - radius_, 
      2*radius_ + 2*hoverBorder, 
      2*radius_ + 2*hoverBorder);
}

void ItemStation::paint(
    QPainter* painter, 
    const QStyleOptionGraphicsItem* option, QWidget* widget)
{
  float x = pose_.position.x;
  float y = pose_.position.y;
  float z = pose_.position.z;

  // draw station rectangle
  painter->setPen(Qt::SolidLine);
  painter->setBrush(*(new QColor(0, 0, 255, 255)));
  painter->drawRect(QRectF(x-radius_/2, y-radius_/2, radius_, radius_));

  // circle around hovered (active) stations
  if (is_hovered_)
  {
    painter->setPen(Qt::SolidLine);
    painter->setBrush(*(new QColor(0, 0, 0, 0)));
    painter->drawEllipse(QPointF(x, y), radius_*2, radius_*2);
  }

  // debug bounding rect:
  if (drawBoundingRect_)
  {
    painter->setBrush(*(new QColor(0, 255, 0, 0)));
    painter->drawRect(this->boundingRect());
  }
}

void ItemStation::setStationName(QString station_name)
{
  station_name_ = station_name;
  setText(station_name);
}

QString ItemStation::getStationName()
{
  return station_name_;
}

void ItemStation::setPose(geometry_msgs::Pose pose)
{
  pose_ = pose;

  setTransformOriginPoint(QPointF(pose_.position.x, pose_.position.y));
}

geometry_msgs::Pose ItemStation::getPose()
{
  return pose_;
}

int ItemStation::getId()
{
  return id_;
}

void ItemStation::setId(int id)
{
  id_ = id;
}

void ItemStation::mousePressEvent(QGraphicsSceneMouseEvent*)
{
}
void ItemStation::mouseMoveEvent(QGraphicsSceneMouseEvent*)
{
}
void ItemStation::mouseReleaseEvent(QGraphicsSceneMouseEvent*)
{
}
void ItemStation::hoverEnterEvent(QGraphicsSceneHoverEvent*)
{
  is_hovered_ = true;
  emit setActiveStation(station_name_.toStdString());
  update();
}
void ItemStation::hoverLeaveEvent(QGraphicsSceneHoverEvent*)
{
  is_hovered_ = false;
  emit setActiveStation("");
  update();
}

void ItemStation::setDrawBoundingRect(bool drawBoundingRect)
{
  drawBoundingRect_ = drawBoundingRect;
}

}  // end namespace tuw_rqt_ordermanager
