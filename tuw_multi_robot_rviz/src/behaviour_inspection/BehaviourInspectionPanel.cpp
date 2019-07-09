#include <include/tuw_multi_robot_rviz/behaviour_inspection/BehaviourInspectionPanel.h>
#include <QtWidgets/QHeaderView>
#include <Qt>

namespace tuw_multi_robot_rviz {

    BehaviourInspectionPanel::BehaviourInspectionPanel(QWidget *parent) : Panel(parent)
    {
        topic_layout = new QHBoxLayout;

        //auto* label = new QLabel( "Output Topic:" );
        table = new QTableWidget(4, 3);

        topic_layout->addWidget(table);
        topic_layout->setMargin(0);
        topic_layout->setAlignment(table, Qt::Alignment::enum_type::AlignTop);
        setLayout(topic_layout);
    }

    void BehaviourInspectionPanel::load(const rviz::Config &config)
    {
        Panel::load(config);
    }

    void BehaviourInspectionPanel::save(rviz::Config config) const
    {
        Panel::save(config);
    }

    void BehaviourInspectionPanel::resizeEvent(QResizeEvent *event)
    {
    }


}

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(tuw_multi_robot_rviz::BehaviourInspectionPanel, rviz::Panel);