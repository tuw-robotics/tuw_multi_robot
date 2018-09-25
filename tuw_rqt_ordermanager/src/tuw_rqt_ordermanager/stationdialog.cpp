#include "tuw_rqt_ordermanager/stationdialog.h"
#include "tuw_rqt_ordermanager/ui_stationdialog.h"

StationDialog::StationDialog(QWidget* parent) : QDialog(parent), ui_(new Ui::StationDialog)
{
  ui_->setupUi(this);

  connect(ui_->buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
  connect(ui_->buttonBox, SIGNAL(rejected()), this, SLOT(reject()));
}

StationDialog::~StationDialog()
{
  delete ui_;
}

QString StationDialog::getStationName()
{
  return ui_->le_station_name->text();
}

int StationDialog::getStationId()
{
  return ui_->le_station_id->text().toInt();
}

float StationDialog::getPositionX()
{
  return ui_->le_pos_x->text().toFloat();
}

float StationDialog::getPositionY()
{
  return ui_->le_pos_y->text().toFloat();
}

float StationDialog::getPositionZ()
{
  return ui_->le_pos_z->text().toFloat();
}

void StationDialog::setStationName(QString name)
{
  ui_->le_station_name->setText(name);
}

void StationDialog::setStationId(int id)
{
  ui_->le_station_id->setText(QString::number(id));
}

void StationDialog::setPositionX(float x)
{
  ui_->le_pos_x->setText(QString::number(x));
}

void StationDialog::setPositionY(float y)
{
  ui_->le_pos_y->setText(QString::number(y));
}

void StationDialog::setPositionZ(float z)
{
  ui_->le_pos_z->setText(QString::number(z));
}
