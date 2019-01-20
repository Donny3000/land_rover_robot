/*
 *
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <ros/console.h>


class TeleopAutoRover
{
    public:
        TeleopAutoRover();

    private:
        void JoyCallback(const sensor_msgs::Joy::ConstPtr& joy);
        void Publish();
  
        ros::NodeHandle ph_, nh_;

        int linear_axis_, angular_axis_, enable_axis_;
        int start_press_cnt_;
        double l_scale_, a_scale_;
        ros::Publisher vel_pub_;
        ros::Subscriber joy_sub_;

        geometry_msgs::Twist last_published_;
        boost::mutex publish_mutex_;
        bool start_pressed_;
        bool zero_twist_published_;
        // Use a timer to keep a continous stream going
        ros::Timer timer_;
};


TeleopAutoRover::TeleopAutoRover() :
    ph_("~"),
    linear_axis_(1),
    angular_axis_(0),
    // Start button
    enable_axis_(3),
    start_press_cnt_(0),
    l_scale_(1),
    a_scale_(1),
    start_pressed_(false),
    zero_twist_published_(false)
{
    ph_.param("linear_axis", linear_axis_, linear_axis_);
    ph_.param("angular_axis", angular_axis_, angular_axis_);
    ph_.param("enabled_axis", enable_axis_, enable_axis_);
    ph_.param("scale_angular", a_scale_, a_scale_);
    ph_.param("scale_linear", l_scale_, l_scale_);

    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("auto_rover/cmd_vel", 1);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopAutoRover::JoyCallback, this);

    timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&TeleopAutoRover::Publish, this));
}

void TeleopAutoRover::Publish()
{
    boost::mutex::scoped_lock lock(publish_mutex_);

    if (start_pressed_)
    {
        vel_pub_.publish(last_published_);
        zero_twist_published_ = false;
    }
    else if (!start_pressed_ && !zero_twist_published_)
    {
        vel_pub_.publish(*new geometry_msgs::Twist());
        zero_twist_published_ = true;
    }
}

void TeleopAutoRover::JoyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    boost::mutex::scoped_lock(publish_mutex_);

    geometry_msgs::Twist vel;

    vel.linear.x = l_scale_ * joy->axes[linear_axis_];
    vel.angular.z = l_scale_ * joy->axes[angular_axis_];
    last_published_ = vel;

    start_press_cnt_ = start_press_cnt_ + joy->buttons[enable_axis_];
    if (start_press_cnt_++ % 2)
    {
        start_pressed_ = true;
    }
    else
    {
        start_pressed_ = false;
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop_auto_rover");
    TeleopAutoRover teleop_auto_rover;

    ros::spin();
}
