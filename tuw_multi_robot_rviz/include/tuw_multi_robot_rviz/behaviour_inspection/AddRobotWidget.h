//
// Created by axelbr on 10.07.19.
//

#ifndef SRC_ADD_ROBOT_WIDGET_H
#define SRC_ADD_ROBOT_WIDGET_H

#include <QtWidgets/QWidget>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QHBoxLayout>

namespace tuw_multi_robot_rviz {

    class AddRobotWidget : public QWidget {
    Q_OBJECT
    public:
        explicit AddRobotWidget(QWidget *parent = nullptr);

    Q_SIGNALS:

        void submittedRobotName(const std::string &robot_name);

    private:
        QLineEdit *robot_name;
        QPushButton *add_button;

        void onClick();

        void onTextChanged();

        void setupWidgets(QHBoxLayout *layout);
    };

}


#endif //SRC_ADD_ROBOT_WIDGET_H
