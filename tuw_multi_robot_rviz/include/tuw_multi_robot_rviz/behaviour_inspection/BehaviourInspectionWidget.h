//
// Created by axelbr on 09.07.19.
//

#ifndef SRC_BEHAVIOURINSPECTIONWIDGET_H
#define SRC_BEHAVIOURINSPECTIONWIDGET_H

#include <QtWidgets/QWidget>
#include <QtWidgets/QTableWidget>
#include <string>
#include <ros/ros.h>

namespace tuw_multi_robot_rviz {
    class BehaviourInspectionWidget : public QWidget {
    Q_OBJECT
    public:
        explicit BehaviourInspectionWidget(const std::vector<std::string> &robot_ids);

        BehaviourInspectionWidget();

    private:
        QTableWidget *table;
        ros::NodeHandle node_handle;
    };
}

#endif //SRC_BEHAVIOURINSPECTIONWIDGET_H
