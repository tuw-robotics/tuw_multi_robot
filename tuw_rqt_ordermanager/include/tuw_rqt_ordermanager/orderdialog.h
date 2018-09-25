#ifndef ORDERDIALOG_H
#define ORDERDIALOG_H

#include <iostream>
#include <QDialog>
#include <QString>

namespace Ui
{
class OrderDialog;
}

class OrderDialog : public QDialog
{
  Q_OBJECT

public:
  explicit OrderDialog(QWidget* parent = 0);
  ~OrderDialog();

  QString getOrderName();
  float getPositionX();
  float getPositionY();
  float getPositionZ();

  void setOrderName(QString);
  void setPositionX(float);
  void setPositionY(float);
  void setPositionZ(float);

private:
  Ui::OrderDialog* ui_;
};

#endif  // ORDERDIALOG_H
