#include "tuw_rqt_ordermanager/robotdialog.h"
#include "tuw_rqt_ordermanager/ui_robotdialog.h"

RobotDialog::RobotDialog(QWidget* parent) : QDialog(parent), ui_(new Ui::RobotDialog)
{
  ui_->setupUi(this);

  connect(ui_->buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
  connect(ui_->buttonBox, SIGNAL(rejected()), this, SLOT(reject()));
}

RobotDialog::~RobotDialog()
{
  delete ui_;
}

QString RobotDialog::getRobotName()
{
  return ui_->le_robot_name->text();
}
float RobotDialog::getPositionX()
{
  return ui_->le_pos_x->text().toFloat();
}
float RobotDialog::getPositionY()
{
  return ui_->le_pos_y->text().toFloat();
}
float RobotDialog::getPositionZ()
{
  return ui_->le_pos_z->text().toFloat();
}

void RobotDialog::setRobotName(QString name)
{
  ui_->le_robot_name->setText(name);
}
void RobotDialog::setPositionX(float x)
{
  ui_->le_pos_x->setText(QString::number(x));
}
void RobotDialog::setPositionY(float y)
{
  ui_->le_pos_y->setText(QString::number(y));
}
void RobotDialog::setPositionZ(float z)
{
  ui_->le_pos_z->setText(QString::number(z));
}
