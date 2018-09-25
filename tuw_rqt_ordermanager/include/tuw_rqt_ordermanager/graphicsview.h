#ifndef TUW_RQT_GRAPHICSVIEW_H
#define TUW_RQT_GRAPHICSVIEW_H

#include <QWidget>
#include <QGraphicsView>
#include <QMenu>
#include <tuw_rqt_ordermanager/map_transformation.h>
#include <tuw_rqt_ordermanager/item_station.h>

namespace tuw_rqt_ordermanager
{
class GraphicsView : public QGraphicsView
{
  Q_OBJECT

public:
  explicit GraphicsView(QWidget* parent = 0);
  void setMapTransformation(MapTransformation* map_transformation);

signals:
  //void orderAddPose(float x, float y, float z);
  void orderAddStation(std::string);
  void contextMenuSignal(QPoint&);
  void newStation(float x, float y, float z);
  void removeStation(std::string);

public slots:
  void wheelEvent(QWheelEvent* event);
  void scalingTime(qreal x);
  void animFinished();
  void showContextMenu(QPoint&);
  void setActiveStation(std::string);

private:
  virtual void mouseMoveEvent(QMouseEvent* event);
  virtual void mousePressEvent(QMouseEvent* event);
  virtual void mouseReleaseEvent(QMouseEvent* event);

  MapTransformation* map_transformation_;

  void addStation(QPointF&);
  void delStation(std::string);

  bool pan_;
  bool moved_;
  int pan_start_x_, pan_start_y_;
  std::string active_station_;

  qreal num_scheduled_scalings_;
  QPoint wheel_event_mouse_pos_;
};

}  // namespace tuw_rqt_ordermanager

#endif
