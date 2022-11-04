#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <can_plugins/Frame.h>
#include <can_utils_rev.hpp>
#include "array"
#include "numeric"

class UmbrellaUnfolder
{
public:
    // bool shirasuInitilization{false}; // falseでAボタン、trueでBボタンが押された状態である。
    float forwardMotorSpeed{3.1419 * 2 * 1}; //記入。[rad/s]

    float reverseMotorDisplacementDestined{3.14159 * 2 * 4}; //目標値記入.[rad]
    float reverseMotorDisplacementCurrent{0.0};

    int motorId{}; //記入

    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
    {

        // else
        // {
        //     can_plugins::Frame frame = can_utils::makeFrame<>(motorId, can_utils::Command::recover_position);
        //     shirasuInitilization = true;
        //     canPub.publish(frame);
        // }

        enum class ShirasuMode : int
        {
            disable,
            homing,
            position,
        };
        ShirasuMode ShirasuModeState{ShirasuMode::disable}; //

        if (joy->buttons[0] && (ShirasuModeState == ShirasuMode::disable || ShirasuModeState == ShirasuMode::position)) // Aボタンを1回押すと、完全に展開。順方向。リミットスイッチによる停止処理が不明！！
        {
            // if (shirasuInitilization)
            // {
            //     can_plugins::Frame frame = can_utils::makeFrame<>(motorId, can_utils::Command::home);
            //     shirasuInitilization = true;
            //     canPub.publish(frame);
            // }

            ShirasuModeState = ShirasuMode::homing;

            can_plugins::Frame frame = can_utils::makeFrame<>(motorId, can_utils::Command::home);
            canPub.publish(frame);

            // hvlで設定？？
            //  can_plugins::Frame frame = can_utils::makeFrame<float>(motorId, forwardMotorSpeed); // 実際にはmotorId+1。
            //  canPub.publish(frame);
        }
        else if (joy->buttons[1] && ShirasuModeState == ShirasuMode::position) // Bボタンを1回押すと、指定する程度畳む。逆方向。順方向動作中にボタンを押さないこと。
        {
            // if (!shirasuInitilization)
            // {
            //     can_plugins::Frame frame = can_utils::makeFrame<>(motorId, can_utils::Command::recover_position);
            //     shirasuInitilization = true;
            //     canPub.publish(frame);
            // }
            ShirasuModeState = ShirasuMode::position;

            can_plugins::Frame frame = can_utils::makeFrame<>(motorId, can_utils::Command::recover_position);
            canPub.publish(frame);

            timerForReverse.start();
        }
    }
    void timerForReverse_callback(const ros::TimerEvent&)
    {
        if (reverseMotorDisplacementCurrent < reverseMotorDisplacementDestined)
        {
            reverseMotorDisplacementCurrent += (reverseMotorDisplacementDestined / 10) * timerForReverse_callback_peorid; // 10sで傘を閉じれると仮定した。
            can_plugins::Frame frame = can_utils::makeFrame<>(motorId, (-1) * reverseMotorDisplacementCurrent);
        }
        else
        {
            reverseMotorDisplacementCurrent = reverseMotorDisplacementDestined;
            can_plugins::Frame frame = can_utils::makeFrame<>(motorId, (-1) * reverseMotorDisplacementCurrent);
            timerForReverse.stop();
        }
    }

    UmbrellaUnfolder(ros::NodeHandle& nh)
        : nh_(nh)
    {
        joySub = nh_.subscribe("joy", 1, &UmbrellaUnfolder::joyCallback, this);
        canPub = nh_.advertise<can_plugins::Frame>("can_tx", 1);
        timerForReverse = nh_.createTimer(ros::Duration(timerForReverse_callback_peorid), &UmbrellaUnfolder::timerForReverse_callback, this, false, false); // 1/ros::Duration()の引数が実行回数毎秒。
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber joySub;
    ros::Publisher canPub;
    ros::Timer timerForReverse;                  //
    double timerForReverse_callback_peorid{0.1}; //設定！
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "umbrella_unfolder");
    ros::NodeHandle nh;
    UmbrellaUnfolder UmbrellaUnfolder{nh}; //?

    ROS_INFO("umbrella_unfolder node has started.");

    ros::spin();

    ROS_INFO("umbrella_unfolder node has terminated.");
    return 0;
}
