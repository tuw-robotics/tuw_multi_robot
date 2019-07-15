#ifndef SRC_BEHAVIOURINSPECTIONPANEL_H
#define SRC_BEHAVIOURINSPECTIONPANEL_H

#include <rviz/panel.h>
#include <QtWidgets/QHBoxLayout>
#include <tuw_multi_robot_msgs/BehaviourProfile.h>
#include "ProfileTableWidget.h"
#include "AddRobotWidget.h"
#include <unordered_map>
#include <ros/ros.h>
#include <boost/optional.hpp>

namespace tuw_multi_robot_rviz {

    class BehaviourInspectionPanel : public rviz::Panel {

    Q_OBJECT
    public:
        explicit BehaviourInspectionPanel(QWidget *parent = nullptr);

        void load(const rviz::Config &config) override;

        void save(rviz::Config config) const override;

    private:
        ProfileTableWidget *profile_table;

        ros::NodeHandle node_handle;

        std::unordered_map<std::string, boost::optional<tuw_multi_robot_msgs::BehaviourProfile>> profile_map;
        std::unordered_map<std::string, ros::Subscriber> subscribers_map;

        void onRobotAdded(const std::string &robot_name);

        void onRobotRemoved(const std::string &robot_name);

        void updateTable();

        ProfileTableEntry mapProfileToEntry(const tuw_multi_robot_msgs::BehaviourProfile &profile);
        void setupWidgets();
    };

}

#endif //SRC_BEHAVIOURINSPECTIONPANEL_H
