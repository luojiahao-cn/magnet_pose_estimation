#ifndef MAGNETIC_FIELD_PANEL_HPP
#define MAGNETIC_FIELD_PANEL_HPP

#include <mag_sensor_node/MagSensorData.h>
#include <mag_sensor_node/MagnetPose.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <rviz/panel.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

#include <QLabel>
#include <QPushButton>
#include <QString>
#include <QVBoxLayout>
#include <QVector>

namespace mag_pose_visualization
{

    class MagneticFieldPanel : public rviz::Panel
    {
        Q_OBJECT
    public:
        MagneticFieldPanel(QWidget *parent = 0);
        ~MagneticFieldPanel() override;

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
        ros::Subscriber magnetic_field_sub_;
        // 统一使用原始传感器数据消息
        void onMagneticFieldMsg(const mag_sensor_node::MagSensorData::ConstPtr &msg);

        ros::Subscriber magnet_pose_sub_;
        QLabel *pose_value_labels_[7];
        void onMagnetPoseMsg(const mag_sensor_node::MagnetPose::ConstPtr &msg);

        QPushButton *switch_topic_button_;
        QString current_topic_ = "/magnetic_field/raw_data";
    };

} // namespace mag_pose_visualization

#endif // MAGNETIC_FIELD_PANEL_HPP
