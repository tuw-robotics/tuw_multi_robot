#ifndef STATIONDIALOG_H
#define STATIONDIALOG_H

#include <iostream>
#include <QDialog>
#include <QString>

namespace Ui
{
class StationDialog;
}

class StationDialog : public QDialog
{
  Q_OBJECT

public:
  explicit StationDialog(QWidget* parent = 0);
  ~StationDialog();

  QString getStationName();
  int getStationId();
  float getPositionX();
  float getPositionY();
  float getPositionZ();

  void setStationName(QString);
  void setStationId(int);
  void setPositionX(float);
  void setPositionY(float);
  void setPositionZ(float);

private:
  Ui::StationDialog* ui_;
};

#endif  // STATIONDIALOG_H
