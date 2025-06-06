#ifndef MAGNETIC_FIELD_PANEL_HPP
#define MAGNETIC_FIELD_PANEL_HPP

#include <ros/ros.h>
#include <rviz/panel.h>
#include <QPushButton>
#include <QVBoxLayout>
#include <QString>
#include <QLabel>
#include <QVector>
#include <ros/subscriber.h>
#include "magnetic_pose_estimation/MagneticField.h"
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <pluginlib/class_list_macros.h>
#include <QLabel>
#include <ros/ros.h>
#include <magnetic_pose_estimation/MagnetPose.h>

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
        void onSwitchTopicClicked(); 

    private:
        ros::NodeHandle nh_;

        QPushButton *calibrate_button_;
        QPushButton *restore_button_;
        QPushButton *reset_localization_button_;
        ros::ServiceClient calibrate_client_;
        ros::ServiceClient reset_client_;
        ros::ServiceClient reset_localization_client_;

        QVector<QLabel *> sensor_value_labels_;
        ros::Subscriber magnetic_field_sub_; // 订阅磁场话题

        void onMagneticFieldMsg(const magnetic_pose_estimation::MagneticField::ConstPtr &msg);

        ros::Subscriber magnet_pose_sub_;
        QLabel *pose_value_labels_[7]; // 每个位姿分量和强度一个数值框
        void onMagnetPoseMsg(const magnetic_pose_estimation::MagnetPose::ConstPtr &msg);

        QPushButton *switch_topic_button_; // 切换按钮
        QString current_topic_ = "/magnetic_field/raw_data"; // 当前订阅的话题
    };

} // end namespace magnetic_pose_estimation

#endif // MAGNETIC_FIELD_PANEL_HPP