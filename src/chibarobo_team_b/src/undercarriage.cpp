// test

#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include <can_plugins/Frame.h>
#include <can_utils_rev.hpp>
#include "array"
#include "numeric"

class Moter //各モータ
{
public: // privateではない
    int id;
    std::array<int, 2> direction;
    float distance{0.05};                    //機体中心からタイヤまでの距離。使われていない！！
    static constexpr double linear_rate{1};  //全インスタンス共通で調整。
    static constexpr double angular_rate{1}; //同上。

public:
    Moter(const uint16_t id, const int distance, const std::array<int, 2> direction)
        : id(id), distance(distance), direction(direction) {} //シラスのid,中心からタイヤまでの距離、タイヤの向いている方向

    //タイヤ表面の速度。タイヤの角速度にしたいなら、タイヤ半径distanceで割る？
    double calcutateSpeed(const std::array<double, 2> &linear, const double &angular) const
    {
        return (linear_rate * std::inner_product(linear.begin(), linear.end(), direction.begin(), 0.0) + angular_rate * angular) / distance; //内積。配列の先頭・末尾のポインタを指定。
    }
};

class Undercarriage
{
public:
    void joyToCanConvertor(const sensor_msgs::Joy::ConstPtr &joy, float sensitivity) //ジョイパッドはXBOXモードで使用する。多分Microsoft Xbox 360 Wired Controller for Linux。左にx軸、上にy軸。
    {
        std::array<double, 2> velocity;                       //機体の並進速度
        double anglerVelocity;                                //機体の回転の角速度
        velocity[0] = (-1) * (joy->axes[6] * sensitivity);    //よこ -1倍する？
        velocity[1] = (1) * (joy->axes[7] * sensitivity);     //たて
        anglerVelocity = (-1) * (joy->axes[3] * sensitivity); //回転

        for (const auto &motor : motors) // range-based for
        {
            can_plugins::Frame frame = can_utils::makeFrame<float>(motor.id, (float)motor.calcutateSpeed(velocity, anglerVelocity));
            canPub.publish(frame);
        }

        canPub.publish(can_utils::makeFrame(0x100, 0x01));
    }
    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
    {
        // // debug用
        // can_plugins::Frame frame = can_utils::makeFrame(777 + 0, 777);
        // canPub.publish(frame);

        // テストはじめ
        // If "START" Pressed, DISABLE>VEL
        if (joy->buttons[7] == 1)
        {
            for (const auto &motor : motors)
            {
                can_plugins::Frame frame = can_utils::makeFrame(motor.id + 0, can_utils::Command::recover_velocity);
                canPub.publish(frame);
            }
        }
        // If "BACK" Pressed,VEL>DISABLE
        else if (joy->buttons[6] == 1)
        {
            for (const auto &motor : motors)
            {
                can_plugins::Frame frame = can_utils::makeFrame(motor.id + 0, can_utils::Command::shutdown);
                canPub.publish(frame);
            }
        }
        //テスト終わり

        joyToCanConvertor(joy, 1 + 0.5 * ((-1) * (joy->axes[5]) + 1)); // RTを押すと、加速度が上がる。oy->axes[5]は、1(RTを押す)~-1(離す)。
    }

    Undercarriage(ros::NodeHandle nh)
        : nh_(nh) //初期化子
    {
        joySub = nh_.subscribe("joy", 1, &Undercarriage::joyCallback, this);
        canPub = nh_.advertise<can_plugins::Frame>("can_tx", 10); // txはtransmit exchange,rxはreceive exchange。

        // デバッグ
        while(true) //!!!!!!!!
        {
            for (const auto &motor : motors)
            {
                can_plugins::Frame frame = can_utils::makeFrame(motor.id + 0, can_utils::Command::recover_velocity);
                canPub.publish(frame);
            }
        }
    }

private:
    std::array<Moter, 4> motors{
        Moter(0x530, 0, {1, 1}),
        Moter(0x000, 0, {-1, 1}),
        Moter(0x000, 0, {-1, -1}),
        Moter(0x000, 0, {1, -1})};
    // double maxWheelVelocity;
    // double maxWheelAcceleration;

    // double wheelRadius;

    uint8_t shirasuMode{0}; // 0>DIS 1>VEL

    ros::NodeHandle nh_;

    ros::Subscriber joySub;
    ros::Publisher canPub;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "undercarriage");
    ros::NodeHandle nh;
    Undercarriage Undercarriage{nh};

    //デバッグ用
    // can_plugins::Frame frame = can_utils::makeFrame(777 + 0, can_utils::Command::shutdown);

    ROS_INFO("undercarriage node has started.");

    // デバッグ用
    // ros::Publisher canPub2 = nh.advertise<can_plugins::Frame>("can_tx", 1);
    // can_plugins::Frame frame = can_utils::makeFrame(12345 + 0, can_utils::Command::shutdown);
    // while (true)
    // {
    //     canPub2.publish(frame);
    // }

    // ros::spin();

    ROS_INFO("undercarriage node has terminated.");
    return 0;
}
