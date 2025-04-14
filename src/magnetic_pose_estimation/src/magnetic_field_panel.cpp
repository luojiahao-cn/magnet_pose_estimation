#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <QVBoxLayout>
#include "magnetic_pose_estimation/magnetic_field_panel.hpp"

namespace magnetic_pose_estimation
{

    MagneticFieldPanel::MagneticFieldPanel(QWidget *parent)
        : rviz::Panel(parent) ,nh_()
    {
        // 创建按钮
        initialize_button_ = new QPushButton("地磁场初始化");
        restore_button_ = new QPushButton("恢复原始数据");

        // 创建水平布局
        QHBoxLayout *layout = new QHBoxLayout;  
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


        ROS_INFO("开始记录初始磁场数据...");
    }

    void MagneticFieldPanel::onRestoreClicked()
    {

        




        ROS_INFO("已恢复原始数据状态");
    }



} // end namespace magnetic_pose_estimation

// 必须保留此宏以注册RViz插件
PLUGINLIB_EXPORT_CLASS(magnetic_pose_estimation::MagneticFieldPanel, rviz::Panel)