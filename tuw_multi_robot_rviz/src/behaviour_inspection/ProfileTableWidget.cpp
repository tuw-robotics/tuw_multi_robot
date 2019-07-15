#include <tuw_multi_robot_rviz/behaviour_inspection/ProfileTableWidget.h>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QTableWidgetItem>
#include <QHBoxLayout>
#include <QString>
#include <iostream>
#include <QtWidgets/QAction>
#include <ros/package.h>

namespace tuw_multi_robot_rviz {

    ProfileTableWidget::ProfileTableWidget()
    {
        auto *layout = new QVBoxLayout;

        setupTable();
        layout->addWidget(table);

        add_robot_widget = new AddRobotWidget(this);
        connect(add_robot_widget, &AddRobotWidget::submittedRobotName, this, &ProfileTableWidget::addRobot);

        layout->addWidget(add_robot_widget);

        layout->setMargin(0);
        layout->setAlignment(table, Qt::Alignment::enum_type::AlignTop);
        layout->setAlignment(add_robot_widget, Qt::Alignment::enum_type::AlignBottom);

        setLayout(layout);
    }

    void ProfileTableWidget::setupTable()
    {

        table = new QTableWidget;
        table->setEditTriggers(QAbstractItemView::NoEditTriggers);
        table->setFocusPolicy(Qt::NoFocus);
        table->setSelectionMode(QAbstractItemView::NoSelection);

        addColumn(Columns::Name, "Robot");
        addColumn(Columns::Profile, "Profile");
        addColumn(Columns::Description, "Description");
        addColumn(Columns::Edit, "Edit");
        table->setColumnWidth(Columns::Edit, 30);
        table->horizontalHeader()->setSectionResizeMode(Columns::Edit, QHeaderView::ResizeMode::Fixed);

        table->verticalHeader()->hide();
        table->horizontalHeader()->setSectionResizeMode(Columns::Description, QHeaderView::Stretch);
    }

    void ProfileTableWidget::update(const std::vector<ProfileTableEntry> &entries)
    {
        table->setRowCount(0);
        for (const auto &entry: entries) {

            int row = table->rowCount();
            table->insertRow(table->rowCount());

            switch (entry.status) {
                case Status::OK:
                    insertActiveRobot(entry, row);
                    break;
                case Status::WAITING:
                    insertInactiveRobot(entry, row);
                    break;

            }
        }
    }

    void ProfileTableWidget::insertActiveRobot(const ProfileTableEntry &entry, int row)
    {
        table->setItem(row, Columns::Name, new QTableWidgetItem(QString::fromStdString(entry.robot)));
        table->setItem(row, Columns::Profile, new QTableWidgetItem(QString::fromStdString(entry.profile)));
        table->setItem(row, Columns::Description, new QTableWidgetItem(QString::fromStdString(entry.description)));

        auto *button = createDeleteButton();
        connect(button, &QPushButton::clicked, [=] { removeRobot(entry.robot); });
        table->setCellWidget(row, Columns::Edit, button);
    }

    void ProfileTableWidget::insertInactiveRobot(const ProfileTableEntry &entry, int row)
    {
        table->setItem(row, Columns::Name, new QTableWidgetItem(QString::fromStdString(entry.robot)));
        table->item(row, Columns::Name)->setBackground(disabled_cell_color);

        table->setItem(row, Columns::Profile, new QTableWidgetItem);
        table->item(row, Columns::Profile)->setBackground(disabled_cell_color);


        table->setItem(row, Columns::Description, new QTableWidgetItem);
        table->item(row, Columns::Description)->setBackground(disabled_cell_color);

        auto *button = createDeleteButton();
        connect(button, &QPushButton::clicked, [=] { removeRobot(entry.robot); });
        table->setCellWidget(row, Columns::Edit, button);
    }

    void ProfileTableWidget::addRobot(const std::string &name)
    {
        Q_EMIT addedRobot(name);
    }

    void ProfileTableWidget::removeRobot(const std::string &robot)
    {

        Q_EMIT removedRobot(robot);
    }

    void ProfileTableWidget::addColumn(int index, const std::string &name)
    {
        table->insertColumn(index);
        table->setHorizontalHeaderItem(index, new QTableWidgetItem(QString::fromStdString(name)));
    }

    QPushButton *ProfileTableWidget::createDeleteButton()
    {
        auto path = ros::package::getPath("tuw_multi_robot_rviz") + "/icons/trash-solid.svg";
        QPushButton *button = new QPushButton;
        button->setIcon(QIcon(QString::fromStdString(path)));
        return button;
    }
}