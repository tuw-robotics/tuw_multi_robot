#ifndef ROBOTDIALOG_H
#define ROBOTDIALOG_H

#include <iostream>
#include <QDialog>
#include <QString>

namespace Ui
{
class RobotDialog;
}

class RobotDialog : public QDialog
{
  Q_OBJECT

public:
  explicit RobotDialog(QWidget* parent = 0);
  ~RobotDialog();

  QString getRobotName();
  float getPositionX();
  float getPositionY();
  float getPositionZ();

  void setRobotName(QString);
  void setPositionX(float);
  void setPositionY(float);
  void setPositionZ(float);

private:
  Ui::RobotDialog* ui_;
};

#endif  // ROBOTDIALOG_H
