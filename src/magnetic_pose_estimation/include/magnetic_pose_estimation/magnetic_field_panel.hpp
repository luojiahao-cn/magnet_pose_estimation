#ifndef MAGNETIC_FIELD_PANEL_HPP
#define MAGNETIC_FIELD_PANEL_HPP

#include <ros/ros.h>
#include <rviz/panel.h>
#include <QPushButton>
#include <QVBoxLayout>
#include <QString>
#include <QLabel>
#include <QMap>
#include <ros/subscriber.h>
#include "magnetic_pose_estimation/MagneticField.h"
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <pluginlib/class_list_macros.h>

namespace magnetic_pose_estimation
{

    class MagneticFieldPanel : public rviz::Panel
    {
        Q_OBJECT
    public:
        MagneticFieldPanel(QWidget *parent = 0);
        virtual ~MagneticFieldPanel();

    protected Q_SLOTS:
        void onCalibrateClicked();
        void onRestoreClicked();
        void onResetLocalizationClicked();

    private:
        ros::NodeHandle nh_;

        QPushButton *calibrate_button_;
        QPushButton *restore_button_;
        QPushButton *reset_localization_button_;
        ros::ServiceClient calibrate_client_;
        ros::ServiceClient reset_client_;
        ros::ServiceClient reset_localization_client_;

        QMap<int, QLabel*> sensor_value_labels_; // 传感器ID到标签的映射
        ros::Subscriber magnetic_field_sub_;     // 订阅磁场话题
        QVBoxLayout *sensor_layout_; // 用于保存传感器数值标签的布局
        void onMagneticFieldMsg(const magnetic_pose_estimation::MagneticField::ConstPtr& msg);
    };

} // end namespace magnetic_pose_estimation

#endif // MAGNETIC_FIELD_PANEL_HPP