#include <include/tuw_multi_robot_rviz/behaviour_inspection/BehaviourInspectionPanel.h>
#include <QtWidgets/QHeaderView>
#include <Qt>

namespace tuw_multi_robot_rviz {

    BehaviourInspectionPanel::BehaviourInspectionPanel(QWidget *parent) : Panel(parent)
    {
        auto *layout = new QVBoxLayout;
        setupWidgets();
        layout->addWidget(profile_table);
        layout->setMargin(5);
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
        auto topic = "/" + robot_name + "/behaviour";
        int buffer_size = 10;
        auto callback = [this, robot_name](const tuw_multi_robot_msgs::BehaviourProfileConstPtr &msg) {
            profile_map[robot_name] = boost::make_optional(*msg);
            ROS_INFO("%s: received msg, size: %i", robot_name.c_str(), (int) profile_map.size());
            updateTable();
        };
        profile_map[robot_name] = boost::optional<tuw_multi_robot_msgs::BehaviourProfile>{};
        auto subscriber = node_handle.subscribe<tuw_multi_robot_msgs::BehaviourProfile>(topic, buffer_size, callback);
        subscribers_map[robot_name] = subscriber;
        updateTable();
    }

    void BehaviourInspectionPanel::onRobotRemoved(const std::string &robot_name)
    {
        profile_map.erase(robot_name);
        subscribers_map[robot_name].shutdown();
        subscribers_map.erase(robot_name);
        updateTable();
    }

    void BehaviourInspectionPanel::updateTable()
    {
        std::vector<ProfileTableEntry> entries;
        std::transform(
                profile_map.begin(),
                profile_map.end(),
                std::back_inserter(entries),
                [&](const std::pair<std::string, boost::optional<tuw_multi_robot_msgs::BehaviourProfile>> &profile) -> ProfileTableEntry {
                    ProfileTableEntry entry;
                    if (profile.second) {
                        entry = mapProfileToEntry(*profile.second);
                    } else {
                        entry.status = Status::WAITING;
                    }
                    entry.robot = profile.first;
                    return entry;
                });
        std::sort(entries.begin(), entries.end(), [](const ProfileTableEntry &lhs, const ProfileTableEntry &rhs) {
            return lhs.robot < rhs.robot;
        });
        ROS_INFO("Entry vector size: %i", (int) entries.size());
        profile_table->update(entries);
    }

    ProfileTableEntry BehaviourInspectionPanel::mapProfileToEntry(const tuw_multi_robot_msgs::BehaviourProfile &profile)
    {
        ProfileTableEntry entry;
        entry.description = profile.description;
        entry.profile = profile.name;
        entry.status = Status::OK;
        return entry;
    }

}

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(tuw_multi_robot_rviz::BehaviourInspectionPanel, rviz::Panel);