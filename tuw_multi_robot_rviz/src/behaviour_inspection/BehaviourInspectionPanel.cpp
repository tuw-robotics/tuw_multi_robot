#include <include/tuw_multi_robot_rviz/behaviour_inspection/BehaviourInspectionPanel.h>
#include <QtWidgets/QHeaderView>
#include <Qt>

namespace tuw_multi_robot_rviz {

    BehaviourInspectionPanel::BehaviourInspectionPanel(QWidget *parent) : Panel(parent)
    {
        auto *layout = new QVBoxLayout;
        setupWidgets();
        layout->addWidget(profile_table);
        layout->setMargin(0);
        layout->setAlignment(profile_table, Qt::Alignment::enum_type::AlignTop);
        setLayout(layout);
    }

    void BehaviourInspectionPanel::setupWidgets()
    {
        profile_table = new ProfileTableWidget;
        connect(profile_table, &ProfileTableWidget::addedRobot, this, &BehaviourInspectionPanel::onRobotAdded);
        connect(profile_table, &ProfileTableWidget::removedRobot, this, &BehaviourInspectionPanel::onRobotRemoved);
    }

    void BehaviourInspectionPanel::load(const rviz::Config &config)
    {
        Panel::load(config);
    }

    void BehaviourInspectionPanel::save(rviz::Config config) const
    {
        Panel::save(config);
    }

    void BehaviourInspectionPanel::onRobotAdded(const std::string &robot_name)
    {
        ProfileTableEntry entry;
        entry.description = "description";
        entry.profile = "slow";

        profile_table->updateRobot(robot_name, entry);
    }

    void BehaviourInspectionPanel::onRobotRemoved(const std::string &robot_name)
    {
        std::cout << "Removed Robot: " << robot_name << std::endl;
    }

}

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(tuw_multi_robot_rviz::BehaviourInspectionPanel, rviz::Panel);