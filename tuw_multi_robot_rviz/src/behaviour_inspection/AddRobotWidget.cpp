//
// Created by axelbr on 10.07.19.
//

#include <tuw_multi_robot_rviz/behaviour_inspection/AddRobotWidget.h>

#include <QHBoxLayout>
#include <iostream>

namespace tuw_multi_robot_rviz {

    AddRobotWidget::AddRobotWidget(QWidget *parent)
    {
        auto *layout = new QHBoxLayout;
        setupWidgets(layout);
        layout->setMargin(0);
        layout->addWidget(add_button);
        setLayout(layout);
    }

    void AddRobotWidget::setupWidgets(QHBoxLayout *layout)
    {
        this->robot_name = new QLineEdit(this);
        this->robot_name->setPlaceholderText("Robot name");
        connect(this->robot_name, &QLineEdit::textChanged, this, &AddRobotWidget::onTextChanged);
        layout->addWidget(this->robot_name);

        this->add_button = new QPushButton(this);
        this->add_button->setText("Add");
        this->add_button->setDisabled(true);
        QObject::connect(this->add_button, &QPushButton::clicked, this, &tuw_multi_robot_rviz::AddRobotWidget::onClick);
    }

    void AddRobotWidget::onTextChanged()
    {
        bool is_empty = robot_name->text().isEmpty();
        this->add_button->setDisabled(is_empty);
    }

    void AddRobotWidget::onClick()
    {
        auto name = robot_name->text().toStdString();
        robot_name->clear();
        Q_EMIT submittedRobotName(name);
    }

}