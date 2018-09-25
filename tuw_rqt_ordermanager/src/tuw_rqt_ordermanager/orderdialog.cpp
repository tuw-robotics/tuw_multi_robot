#include "tuw_rqt_ordermanager/orderdialog.h"
#include "tuw_rqt_ordermanager/ui_orderdialog.h"

OrderDialog::OrderDialog(QWidget* parent) : QDialog(parent), ui_(new Ui::OrderDialog)
{
  ui_->setupUi(this);

  connect(ui_->buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
  connect(ui_->buttonBox, SIGNAL(rejected()), this, SLOT(reject()));
}

OrderDialog::~OrderDialog()
{
  delete ui_;
}

QString OrderDialog::getOrderName()
{
  return this->ui_->le_order_name->text();
}
float OrderDialog::getPositionX()
{
  return this->ui_->le_pos_x->text().toFloat();
}
float OrderDialog::getPositionY()
{
  return this->ui_->le_pos_y->text().toFloat();
}
float OrderDialog::getPositionZ()
{
  return this->ui_->le_pos_z->text().toFloat();
}

void OrderDialog::setOrderName(QString name)
{
  this->ui_->le_order_name->setText(name);
}
void OrderDialog::setPositionX(float x)
{
  this->ui_->le_pos_x->setText(QString::number(x));
}
void OrderDialog::setPositionY(float y)
{
  this->ui_->le_pos_y->setText(QString::number(y));
}
void OrderDialog::setPositionZ(float z)
{
  this->ui_->le_pos_z->setText(QString::number(z));
}
