#ifndef TUW_RQT_ITEMROBOT_H
#define TUW_RQT_ITEMROBOT_H

#include <QObject>
#include <QGraphicsView>

#include <QGraphicsItem>
#include <QListWidgetItem>

#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <cmath>

namespace tuw_rqt_ordermanager
{
class ItemRobot : public QObject, public QGraphicsItem, public QListWidgetItem
{
  Q_OBJECT
public:
  explicit ItemRobot();
  QRectF boundingRect() const;
  void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget);
  void setRobotName(QString);
  QString getRobotName();
  void setRobotRadius(float);
  void setPose(geometry_msgs::Pose);
  void setDrawBoundingRect(bool);

private:
  QString robot_name_;
  geometry_msgs::Pose pose_;
  float radius_;
  bool drawBoundingRect_;
};

}  // namespace tuw_rqt_ordermanager

#endif
