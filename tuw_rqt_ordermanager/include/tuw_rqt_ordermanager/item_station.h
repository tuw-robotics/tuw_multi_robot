#ifndef TUW_RQT_ITEMSTATION_H
#define TUW_RQT_ITEMSTATION_H

#include <QObject>
#include <QGraphicsView>

#include <QGraphicsItem>
#include <QListWidgetItem>

#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <cmath>

namespace tuw_rqt_ordermanager
{
class ItemStation : public QObject, public QGraphicsItem, public QListWidgetItem
{
  Q_OBJECT
public:
  explicit ItemStation();
  QRectF boundingRect() const;
  void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget);
  void setStationName(QString);
  QString getStationName();
  void setPose(geometry_msgs::Pose);
  geometry_msgs::Pose getPose();
  void setId(int);
  int getId();
  void setDrawBoundingRect(bool);

signals:
  void setActiveStation(std::string);

private:
  QString station_name_;
  geometry_msgs::Pose pose_;
  float radius_;
  int id_;
  bool is_hovered_;
  bool drawBoundingRect_;

protected:
  void mousePressEvent(QGraphicsSceneMouseEvent*);
  void mouseMoveEvent(QGraphicsSceneMouseEvent*);
  void mouseReleaseEvent(QGraphicsSceneMouseEvent*);
  void hoverEnterEvent(QGraphicsSceneHoverEvent*);
  void hoverLeaveEvent(QGraphicsSceneHoverEvent*);
};

}  // namespace tuw_rqt_ordermanager

#endif

