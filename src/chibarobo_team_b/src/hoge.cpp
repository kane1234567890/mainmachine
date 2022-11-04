#include <ros/ros.h>
#include <can_plugins/Frame.h>
// #include <can_plugins/can_util.hpp>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "undercarriage");
    ros::NodeHandle nh;
    // Undercarriage Undercarriage{nh};
    can_plugins::Frame frame = can_plugins::Frame{};
    frame.id =  777 + 0;
    frame.data[0] =  5;
    frame.dlc = 1;

    ROS_INFO("undercarriage node has started.");
    ros::Publisher canPub2 = nh.advertise<can_plugins::Frame>("can_tx", 1);

    while (true)
    {
        canPub2.publish(frame);
    }

    // ros::spin();

    ROS_INFO("undercarriage node has terminated.");
    return 0;
}