#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <can_plugins/Frame.h>
#include <can_utils_rev.hpp>
#include "array"
#include "numeric"

class BallEjector
{
public:
    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
    {
        if (joy->buttons[2]) // Xボタンを押すと、発射。順方向。
        {
            joyToCanConvertor(joy, 1);
        }
        if (joy->buttons[3]) // Yボタンを押すと、発射やめ。逆方向。
        {
            joyToCanConvertor(joy, 0);
        }
    }
    void joyToCanConvertor(const sensor_msgs::Joy::ConstPtr &joy, int boolValue)
    {
        int motorId{0x500}; //記入

        if (boolValue == 1)
        {
            can_plugins::Frame frame = can_utils::makeFrame<>(motorId, can_utils::BaseBoardCommand::frot);
            canPub.publish(frame);
        }
        if (boolValue == 0)
        {
            can_plugins::Frame frame = can_utils::makeFrame<>(motorId, can_utils::BaseBoardCommand::brot);
            canPub.publish(frame);
        }
    }

    BallEjector(ros::NodeHandle nh)
        : nh_(nh)
    {
        joySub = nh_.subscribe("joy", 1, &BallEjector::joyCallback, this);
        canPub = nh_.advertise<can_plugins::Frame>("can_tx", 1);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber joySub;
    ros::Publisher canPub;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ball_ejector");
    ros::NodeHandle nh;
    BallEjector BallEjector{nh}; //?

    ROS_INFO("ball_ejector node has started.");

    ros::spin();

    ROS_INFO("ball_ejector node has terminated.");
    return 0;
}
