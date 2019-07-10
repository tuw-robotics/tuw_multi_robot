#include <tuw_multi_robot_rviz/behaviour_inspection/ProfileTableWidget.h>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QTableWidgetItem>
#include <QHBoxLayout>
#include <QString>
#include <iostream>

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

        table->verticalHeader()->hide();
        table->horizontalHeader()->setSectionResizeMode(Columns::Description, QHeaderView::Stretch);
    }

    void ProfileTableWidget::addRobot(const std::string &name)
    {
        auto found = std::find(robots.begin(), robots.end(), name);
        if (found == robots.end()) {
            robots.push_back(name);
            int current_row = addRow();

            table->item(current_row, Columns::Name)->setText(QString::fromStdString(name));

            QPushButton *button = new QPushButton("X");
            connect(button, &QPushButton::clicked, [=] { removeRobot(name); });
            table->setCellWidget(current_row, Columns::Edit, button);

            Q_EMIT addedRobot(name);
        }
    }

    void ProfileTableWidget::updateRobot(const std::string &robot, ProfileTableEntry entry)
    {
        auto found = std::find(robots.begin(), robots.end(), robot);

        if (found != robots.end()) {
            int row = std::distance(robots.begin(), found);
            writeEntry(row, entry);
        }
    }

    void ProfileTableWidget::removeRobot(const std::string &robot)
    {
        auto found = std::find(robots.begin(), robots.end(), robot);
        if (found != robots.end()) {
            table->removeRow(std::distance(robots.begin(), found));
            robots.erase(found);
            Q_EMIT removedRobot(robot);
        }
    }

    int ProfileTableWidget::addRow()
    {
        int row = table->rowCount();
        table->insertRow(row);
        for (int i = 0; i < table->columnCount(); i++) {
            table->setItem(row, i, new QTableWidgetItem);
        }
        return row;
    }

    void ProfileTableWidget::addColumn(int index, const std::string &name)
    {
        table->insertColumn(index);
        table->setHorizontalHeaderItem(index, new QTableWidgetItem(QString::fromStdString(name)));
    }


    void ProfileTableWidget::writeEntry(int row, ProfileTableEntry entry)
    {
        table->item(row, Columns::Profile)->setText(QString::fromStdString(entry.profile));
        table->item(row, Columns::Description)->setText(QString::fromStdString(entry.description));
    }
}