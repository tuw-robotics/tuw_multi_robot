//
// Created by axelbr on 09.07.19.
//

#ifndef SRC_PROFILETABLEWIDGET_H
#define SRC_PROFILETABLEWIDGET_H

#include <QtWidgets/QWidget>
#include <QtWidgets/QTableWidget>
#include <string>
#include <set>
#include "AddRobotWidget.h"

namespace tuw_multi_robot_rviz {

    struct ProfileTableEntry {
        std::string profile;
        std::string description;
    };

    class ProfileTableWidget : public QWidget {
    Q_OBJECT
    public:
        ProfileTableWidget();

        void updateRobot(const std::string &robot, ProfileTableEntry entry);

        void removeRobot(const std::string &robot);

        void addRobot(const std::string &name);

    Q_SIGNALS:

        void addedRobot(const std::string &name);

        void removedRobot(const std::string &name);

    private:
        QTableWidget *table;
        AddRobotWidget *add_robot_widget;

        std::set<std::string> robots;

        int addRow();

        void addColumn(int index, const std::string &name);

        void writeEntry(int row, ProfileTableEntry entry);

        void setupTable();

        enum Columns {
            Name = 0,
            Profile,
            Description,
            Edit
        };
    };
}

#endif //SRC_PROFILETABLEWIDGET_H
