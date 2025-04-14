#ifndef MAGNETIC_FIELD_PANEL_HPP
#define MAGNETIC_FIELD_PANEL_HPP

#include <ros/ros.h>
#include <rviz/panel.h>
#include <QPushButton>
#include <QVBoxLayout>
#include <magnetic_pose_estimation/MagneticField.h>

namespace magnetic_pose_estimation
{

    class MagneticFieldPanel : public rviz::Panel
    {
        Q_OBJECT
    public:
        MagneticFieldPanel(QWidget *parent = 0);
        virtual ~MagneticFieldPanel();

    protected Q_SLOTS:
        void onInitializeClicked();
        void onRestoreClicked();

    protected:
        void magneticFieldCallback(const magnetic_pose_estimation::MagneticField::ConstPtr &msg);

    private:
        ros::NodeHandle nh_;
        ros::Subscriber magnetic_field_sub_;
        std::map<int, magnetic_pose_estimation::MagneticField> initial_readings_;

        QPushButton *initialize_button_;
        QPushButton *restore_button_;
        bool is_initializing_;
    };

} // end namespace magnetic_pose_estimation

#endif // MAGNETIC_FIELD_PANEL_HPP