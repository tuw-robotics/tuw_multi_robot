//
// Created by axelbr on 09.07.19.
//

#ifndef SRC_PROFILETABLEWIDGET_H
#define SRC_PROFILETABLEWIDGET_H

#include <QtWidgets/QWidget>
#include <QtWidgets/QTableWidget>
#include <string>
#include "AddRobotWidget.h"
#include <vector>

namespace tuw_multi_robot_rviz {

    enum class Status {
        OK,
        WAITING
    };

    struct ProfileTableEntry {
        std::string robot;
        std::string profile;
        std::string description;
        Status status;
    };

    class ProfileTableWidget : public QWidget {
    Q_OBJECT
    public:
        ProfileTableWidget();

        void update(const std::vector<ProfileTableEntry> &entries);

        void removeRobot(const std::string &robot);

        void addRobot(const std::string &name);

    Q_SIGNALS:

        void addedRobot(const std::string &name);

        void removedRobot(const std::string &name);

    private:
        QTableWidget *table;
        AddRobotWidget *add_robot_widget;

        std::vector<std::string> robots;

        const QColor disabled_cell_color = QColor::fromRgb(237, 237, 237);

        enum Columns {
            Name = 0,
            Profile = 1,
            Description = 2,
            Edit = 3
        };

        void insertActiveRobot(const ProfileTableEntry &entry, int row);

        void insertInactiveRobot(const ProfileTableEntry &entry, int row);

        void addColumn(int index, const std::string &name);

        QPushButton *createDeleteButton();

        void setupTable();
    };
}

#endif //SRC_PROFILETABLEWIDGET_H
