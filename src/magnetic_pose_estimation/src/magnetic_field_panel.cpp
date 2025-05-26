#include "magnetic_pose_estimation/magnetic_field_panel.hpp"
#include <QLabel>
#include <QVector>
#include <cmath>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include "magnetic_pose_estimation/MagnetPose.h"

namespace magnetic_pose_estimation
{

    MagneticFieldPanel::MagneticFieldPanel(QWidget *parent)
        : rviz::Panel(parent), nh_()
    {
        calibrate_button_ = new QPushButton("地磁场校准");
        restore_button_ = new QPushButton("恢复原始数据");
        reset_localization_button_ = new QPushButton("定位重置");

        QHBoxLayout *layout = new QHBoxLayout;
        layout->addWidget(calibrate_button_);
        layout->addWidget(restore_button_);
        layout->addWidget(reset_localization_button_);

        QVBoxLayout *main_layout = new QVBoxLayout;
        main_layout->addLayout(layout);

        // ----------- 分割线 -----------
        QFrame *splitter1 = new QFrame();
        splitter1->setFrameShape(QFrame::HLine);
        splitter1->setFrameShadow(QFrame::Sunken);
        splitter1->setStyleSheet("color: #cccccc; background: #cccccc; min-height: 2px;");
        main_layout->addWidget(splitter1);

        // ----------- 位姿表格 -----------
        QWidget *pose_widget = new QWidget;
        QGridLayout *pose_layout = new QGridLayout(pose_widget);

        QStringList pose_headers = {
            "x (m)", "y (m)", "z (m)",
            "roll (rad)", "pitch (rad)", "yaw (rad)",
            "磁铁强度 (A·m²)"};
        for (int i = 0; i < pose_headers.size(); ++i)
        {
            QLabel *header = new QLabel(pose_headers[i]);
            header->setAlignment(Qt::AlignCenter);
            header->setMinimumWidth(120);
            header->setMinimumHeight(32);
            header->setMaximumHeight(40);
            header->setStyleSheet("font-weight: bold;");
            pose_layout->addWidget(header, 0, i);
        }
        for (int i = 0; i < 7; ++i)
        {
            pose_value_labels_[i] = new QLabel("--");
            pose_value_labels_[i]->setAlignment(Qt::AlignCenter);
            pose_value_labels_[i]->setStyleSheet("border: 1px solid #888; border-radius: 4px; background: #ffffff;");
            pose_value_labels_[i]->setMinimumWidth(120);
            pose_value_labels_[i]->setMinimumHeight(45);
            pose_value_labels_[i]->setMaximumHeight(55);
            pose_layout->addWidget(pose_value_labels_[i], 1, i);
        }
        pose_widget->setMaximumHeight(110);
        main_layout->addWidget(pose_widget);

        // ----------- 分割线 -----------
        QFrame *splitter2 = new QFrame();
        splitter2->setFrameShape(QFrame::HLine);
        splitter2->setFrameShadow(QFrame::Sunken);
        splitter2->setStyleSheet("color: #cccccc; background: #cccccc; min-height: 2px;");
        main_layout->addWidget(splitter2);

        // ----------- 传感器表格 -----------
        QWidget *sensor_widget = new QWidget;
        QGridLayout *sensor_layout = new QGridLayout(sensor_widget);

        sensor_value_labels_.resize(25 * 3);
        for (int i = 0; i < 25; ++i)
        {
            int row = i / 5;
            int col_base = (i % 5) * 4;

            QLabel *label_sensor = new QLabel(QString("%1:").arg(i + 1));
            label_sensor->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
            label_sensor->setStyleSheet("");
            label_sensor->setMinimumWidth(40);
            label_sensor->setMinimumHeight(34);
            label_sensor->setMaximumHeight(44);
            sensor_layout->addWidget(label_sensor, row, col_base);

            for (int j = 0; j < 3; ++j)
            {
                int idx = i * 3 + j;
                QString axis = (j == 0) ? "X" : (j == 1) ? "Y" : "Z";
                sensor_value_labels_[idx] = new QLabel(QString("%1: --").arg(axis));
                sensor_value_labels_[idx]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
                sensor_value_labels_[idx]->setStyleSheet("border: 1px solid gray; border-radius: 4px; background: #ffffff;");
                sensor_value_labels_[idx]->setMinimumWidth(120);
                sensor_value_labels_[idx]->setMinimumHeight(45);
                sensor_value_labels_[idx]->setMaximumHeight(55);
                sensor_layout->addWidget(sensor_value_labels_[idx], row, col_base + j + 1);
            }
        }
        sensor_widget->setMaximumHeight(300);
        main_layout->addWidget(sensor_widget);

        setLayout(main_layout);

        connect(calibrate_button_, SIGNAL(clicked()), this, SLOT(onCalibrateClicked()));
        connect(restore_button_, SIGNAL(clicked()), this, SLOT(onRestoreClicked()));
        connect(reset_localization_button_, SIGNAL(clicked()), this, SLOT(onResetLocalizationClicked()));

        calibrate_client_ = nh_.serviceClient<std_srvs::Empty>("/magnetic_field/calibrate_earth_field");
        reset_client_ = nh_.serviceClient<std_srvs::Empty>("/magnetic_field/reset_to_initial");
        reset_localization_client_ = nh_.serviceClient<std_srvs::Empty>("/magnet_pose/reset_localization");

        magnetic_field_sub_ = nh_.subscribe("/magnetic_field/raw_data", 30, &MagneticFieldPanel::onMagneticFieldMsg, this);
        magnet_pose_sub_ = nh_.subscribe("/magnet_pose/predicted", 10, &MagneticFieldPanel::onMagnetPoseMsg, this);
    }

    MagneticFieldPanel::~MagneticFieldPanel()
    {
    }

    void MagneticFieldPanel::onCalibrateClicked()
    {
        ROS_INFO("开始地磁场校准...");
        std_srvs::Empty srv;
        if (calibrate_client_.call(srv))
            ROS_INFO("地磁场校准服务已调用");
        else
            ROS_ERROR("地磁场校准服务调用失败！");
    }

    void MagneticFieldPanel::onRestoreClicked()
    {
        ROS_INFO("请求恢复原始数据...");
        std_srvs::Empty srv;
        if (reset_client_.call(srv))
            ROS_INFO("已恢复原始数据状态");
        else
            ROS_ERROR("恢复原始数据服务调用失败！");
    }

    void MagneticFieldPanel::onResetLocalizationClicked()
    {
        ROS_INFO("请求定位重置...");
        std_srvs::Empty srv;
        if (reset_localization_client_.call(srv))
            ROS_INFO("定位重置服务已调用");
        else
            ROS_ERROR("定位重置服务调用失败！");
    }

    void MagneticFieldPanel::onMagneticFieldMsg(const magnetic_pose_estimation::MagneticField::ConstPtr &msg)
    {
        int idx = static_cast<int>(msg->sensor_id) - 1;
        if (idx >= 0 && idx < 25)
        {
            sensor_value_labels_[idx * 3 + 0]->setText(QString("%1").arg(static_cast<int>(msg->mag_x)));
            sensor_value_labels_[idx * 3 + 1]->setText(QString("%1").arg(static_cast<int>(msg->mag_y)));
            sensor_value_labels_[idx * 3 + 2]->setText(QString("%1").arg(static_cast<int>(msg->mag_z)));
        }
    }

    void MagneticFieldPanel::onMagnetPoseMsg(const magnetic_pose_estimation::MagnetPose::ConstPtr &msg)
    {
        double x = msg->position.x;
        double y = msg->position.y;
        double z = msg->position.z;

        double qw = msg->orientation.w;
        double qx = msg->orientation.x;
        double qy = msg->orientation.y;
        double qz = msg->orientation.z;

        // 四元数转欧拉角
        double sinr_cosp = 2 * (qw * qx + qy * qz);
        double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
        double roll = std::atan2(sinr_cosp, cosr_cosp);

        double sinp = 2 * (qw * qy - qz * qx);
        double pitch;
        if (std::abs(sinp) >= 1)
            pitch = std::copysign(M_PI / 2, sinp);
        else
            pitch = std::asin(sinp);

        double siny_cosp = 2 * (qw * qz + qx * qy);
        double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
        double yaw = std::atan2(siny_cosp, cosy_cosp);

        double strength = msg->magnetic_strength;

        pose_value_labels_[0]->setText(QString::number(x, 'f', 5));
        pose_value_labels_[1]->setText(QString::number(y, 'f', 5));
        pose_value_labels_[2]->setText(QString::number(z, 'f', 5));
        pose_value_labels_[3]->setText(QString::number(roll, 'f', 3));
        pose_value_labels_[4]->setText(QString::number(pitch, 'f', 3));
        pose_value_labels_[5]->setText(QString::number(yaw, 'f', 3));
        pose_value_labels_[6]->setText(QString::number(strength, 'f', 2));
    }

} // end namespace magnetic_pose_estimation

PLUGINLIB_EXPORT_CLASS(magnetic_pose_estimation::MagneticFieldPanel, rviz::Panel)