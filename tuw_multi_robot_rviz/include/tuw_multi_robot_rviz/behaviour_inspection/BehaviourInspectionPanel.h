#ifndef SRC_BEHAVIOURINSPECTIONPANEL_H
#define SRC_BEHAVIOURINSPECTIONPANEL_H

#include <rviz/panel.h>
#include <QtWidgets/QHBoxLayout>
#include <tuw_multi_robot_msgs/BehaviourProfile.h>
#include "ProfileTableWidget.h"
#include "AddRobotWidget.h"

namespace tuw_multi_robot_rviz {

    class BehaviourInspectionPanel : public rviz::Panel {

    Q_OBJECT
    public:
        explicit BehaviourInspectionPanel(QWidget *parent = 0);

        void load(const rviz::Config &config) override;

        void save(rviz::Config config) const override;

    private:
        ProfileTableWidget *profile_table;

        void onRobotAdded(const std::string &robot_name);

        void onRobotRemoved(const std::string &robot_name);

        void setupWidgets();
    };

}

#endif //SRC_BEHAVIOURINSPECTIONPANEL_H
