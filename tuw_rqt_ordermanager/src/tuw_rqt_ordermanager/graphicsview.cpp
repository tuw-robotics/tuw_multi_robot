#include "tuw_rqt_ordermanager/graphicsview.h"
#include <QMouseEvent>
#include <QApplication>
#include <QScrollBar>
#include <qmath.h>

#include <QTimeLine>
#include <iostream>

namespace tuw_rqt_ordermanager
{
// https://stackoverflow.com/questions/19113532/qgraphicsview-zooming-in-and-out-under-mouse-position-using-mouse-wheel/50517097#50517097
GraphicsView::GraphicsView(QWidget* parent) : QGraphicsView(parent)
{
  setTransformationAnchor(QGraphicsView::NoAnchor);
  setResizeAnchor(QGraphicsView::NoAnchor);
  num_scheduled_scalings_ = 0;
  active_station_ = "";

  connect(this, SIGNAL(contextMenuSignal(QPoint&)), this, SLOT(showContextMenu(QPoint&)));
  pan_ = false;
  setMouseTracking(true);
}

void GraphicsView::wheelEvent(QWheelEvent* event)
{
  wheel_event_mouse_pos_ = event->pos();

  int numDegrees = event->delta() / 8;
  int numSteps = numDegrees / 15;  // see QWheelEvent documentation
  num_scheduled_scalings_ += numSteps;
  if (num_scheduled_scalings_ * numSteps <
      0)  // if user moved the wheel in another direction, we reset previously scheduled scalings
    num_scheduled_scalings_ = numSteps;

  QTimeLine* anim = new QTimeLine(350, this);
  anim->setUpdateInterval(20);

  connect(anim, SIGNAL(valueChanged(qreal)), SLOT(scalingTime(qreal)));
  connect(anim, SIGNAL(finished()), SLOT(animFinished()));
  anim->start();
  event->accept();
}

void GraphicsView::scalingTime(qreal x)
{
  QPointF oldPos = mapToScene(wheel_event_mouse_pos_);

  qreal factor = 1.0 + qreal(num_scheduled_scalings_) / 300.0;
  scale(factor, factor);

  QPointF newPos = mapToScene(wheel_event_mouse_pos_);
  QPointF delta = newPos - oldPos;
  this->translate(delta.x(), delta.y());
}

void GraphicsView::animFinished()
{
  if (num_scheduled_scalings_ > 0)
    num_scheduled_scalings_--;
  else
    num_scheduled_scalings_++;

  sender()->~QObject();
}

void GraphicsView::mousePressEvent(QMouseEvent* event)
{
  
  if (event->button() == Qt::LeftButton)
  {
    if (active_station_ != "" )
    {
      pan_ = false;
      float x = event->x();
      float y = event->y();
      float z = 0;
      QPointF scenePoint = mapToScene(x, y);
      x = scenePoint.x();
      y = scenePoint.y();
      //emit orderAddPose(x, y, z);
      emit orderAddStation(active_station_);
      event->ignore();
      QGraphicsView::mousePressEvent(event);
      return;
    } 
    else
    {
      QPoint* pos = new QPoint(event->x(), event->y());
      QPointF scenePos = this->mapToScene(*pos);
      QPointF* map_pos = new QPointF(scenePos.x(), scenePos.y());
      float x = map_transformation_->transformSceneToMap(MapTransformation::TRANSFORM_X, scenePos.x());
      float y = map_transformation_->transformSceneToMap(MapTransformation::TRANSFORM_Y, scenePos.y());
      float z = map_transformation_->transformSceneToMap(MapTransformation::TRANSFORM_Z, 0);

      emit newStation(x, y, z);
    }
  }
  else if (event->button() == Qt::RightButton)
  {
    pan_ = true;
    moved_ = false;
    pan_start_x_ = event->x();
    pan_start_y_ = event->y();
    setCursor(Qt::ClosedHandCursor);
    //event->accept();
    return;
  }
  //event->ignore();
}
void GraphicsView::mouseReleaseEvent(QMouseEvent* event)
{
  if (event->button() == Qt::LeftButton)
  {
    event->ignore();
    QGraphicsView::mouseReleaseEvent(event);
    return;
  }
  else if (event->button() == Qt::RightButton)
  {
    if (!moved_)
    {
      //this->showContextMenu();
      QPoint* pos = new QPoint(event->x(), event->y());
      emit contextMenuSignal(*pos);
    }

    pan_ = false;
    setCursor(Qt::ArrowCursor);
    //event->accept();
    event->ignore();
    return;
  }
  //event->ignore();
}

void GraphicsView::addStation(QPointF& pos)
{
  float x = map_transformation_->transformSceneToMap(MapTransformation::TRANSFORM_X, pos.x());
  float y = map_transformation_->transformSceneToMap(MapTransformation::TRANSFORM_Y, pos.y());
  float z = map_transformation_->transformSceneToMap(MapTransformation::TRANSFORM_Z, 0);

  emit newStation(x, y, z);
}

void GraphicsView::delStation(std::string active_station)
{
  if (active_station != "")
    emit removeStation(active_station);
}

void GraphicsView::showContextMenu(QPoint& pos)
{
  // when contextmenu opens, focus leaves station thus active_station_ becomes
  // "". therefore store the current active station for context-menu actions.
  std::string active_station_temp = active_station_;

  QMenu ctxMenu;
  QPoint globalPos = this->mapToGlobal(pos);
  QPointF scenePos = this->mapToScene(pos);
  QPointF* map_pos = new QPointF(scenePos.x(), scenePos.y());
  QAction* addStation = ctxMenu.addAction("add station here");
  QAction* delStation = ctxMenu.addAction("remove station");
  if (active_station_temp == "")
    delStation->setEnabled(false);

  QAction* selected = ctxMenu.exec(globalPos);

  if ( selected == addStation )
    this->addStation(*map_pos);
  else if ( selected == delStation )
    this->delStation(active_station_temp);
}

void GraphicsView::mouseMoveEvent(QMouseEvent* event)
{
  if (pan_)
  {
    horizontalScrollBar()->setValue(horizontalScrollBar()->value() - (event->x() - pan_start_x_));
    verticalScrollBar()->setValue(verticalScrollBar()->value() - (event->y() - pan_start_y_));
    pan_start_x_ = event->x();
    pan_start_y_ = event->y();
    moved_ = true;
    event->accept();
    return;
  }

  QGraphicsView::mouseMoveEvent(event);
  return;
}

void GraphicsView::setMapTransformation(MapTransformation* map_transformation)
{
  map_transformation_ = map_transformation;
}

void GraphicsView::setActiveStation(std::string active_station)
{
  active_station_ = active_station;
}

}  // end namespace tuw_rqt_ordermanager
