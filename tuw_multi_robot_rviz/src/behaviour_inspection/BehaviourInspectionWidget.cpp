#include <tuw_multi_robot_rviz/behaviour_inspection/BehaviourInspectionWidget.h>
#include <QtWidgets/QHeaderView>

namespace tuw_multi_robot_rviz {
    BehaviourInspectionWidget::BehaviourInspectionWidget(const std::vector<std::string> &robot_ids)
    {
        QStringList string_list;
        QString robot = "Robot";
        QString desired = "Profile";
        QString current = "Parameters";

        string_list.append(robot);
        string_list.append(desired);
        string_list.append(current);

        table = new QTableWidget(robot_ids.size(), 3);
        table->setEditTriggers(QAbstractItemView::NoEditTriggers);
        table->setFocusPolicy(Qt::NoFocus);
        table->setSelectionMode(QAbstractItemView::NoSelection);
        table->setHorizontalHeaderLabels(string_list);
        table->verticalHeader()->hide();
        table->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch);

    }

    BehaviourInspectionWidget::BehaviourInspectionWidget() : BehaviourInspectionWidget(std::vector<std::string>())
    {
    }
}