#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <QVBoxLayout>
#include "magnetic_pose_estimation/magnetic_field_panel.hpp"

namespace magnetic_pose_estimation
{

    MagneticFieldPanel::MagneticFieldPanel(QWidget *parent)
        : rviz::Panel(parent), is_initializing_(false)
    {
        // 创建按钮
        initialize_button_ = new QPushButton("地磁场初始化");
        restore_button_ = new QPushButton("恢复原始数据");
        restore_button_->setEnabled(false);

        // 创建垂直布局
        QVBoxLayout *layout = new QVBoxLayout;
        layout->addWidget(initialize_button_);
        layout->addWidget(restore_button_);
        setLayout(layout);

        // 连接按钮信号
        connect(initialize_button_, SIGNAL(clicked()), this, SLOT(onInitializeClicked()));
        connect(restore_button_, SIGNAL(clicked()), this, SLOT(onRestoreClicked()));
    }

    MagneticFieldPanel::~MagneticFieldPanel()
    {
    }

    void MagneticFieldPanel::onInitializeClicked()
    {
        is_initializing_ = true;
        initialize_button_->setEnabled(false);
        restore_button_->setEnabled(true);
        ROS_INFO("开始记录初始磁场数据...");
    }

    void MagneticFieldPanel::onRestoreClicked()
    {
        is_initializing_ = false;
        initialize_button_->setEnabled(true);
        restore_button_->setEnabled(false);
        ROS_INFO("已恢复原始数据状态");
    }

    void MagneticFieldPanel::magneticFieldCallback(
        const magnetic_pose_estimation::MagneticField::ConstPtr &msg)
    {
        // 删除具体实现，仅保留函数框架
    }

} // end namespace magnetic_pose_estimation

// 必须保留此宏以注册RViz插件
PLUGINLIB_EXPORT_CLASS(magnetic_pose_estimation::MagneticFieldPanel, rviz::Panel)