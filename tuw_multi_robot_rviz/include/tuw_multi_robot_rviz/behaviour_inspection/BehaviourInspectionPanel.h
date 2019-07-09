#ifndef SRC_BEHAVIOURINSPECTIONPANEL_H
#define SRC_BEHAVIOURINSPECTIONPANEL_H

#include <rviz/panel.h>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QTableWidget>
#include <tuw_multi_robot_msgs/BehaviourProfile.h>

namespace tuw_multi_robot_rviz {

    class BehaviourInspectionPanel : public rviz::Panel {

    Q_OBJECT
    public:
        explicit BehaviourInspectionPanel(QWidget *parent = 0);

        void load(const rviz::Config &config) override;

        void save(rviz::Config config) const override;

    protected:
        void resizeEvent(QResizeEvent *event) override;

    public Q_SLOTS:

    private:
        QHBoxLayout *topic_layout;
        QTableWidget *table;
    };

}

#endif //SRC_BEHAVIOURINSPECTIONPANEL_H
