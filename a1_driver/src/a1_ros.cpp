//
// Created by sun on 2021/2/24.
//
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "a1_msgs/msg/cmd_mode.hpp"
#include "a1_msgs/msg/cmd_vel.hpp"
#include "driver.h"

using namespace UNITREE_LEGGED_SDK;
static int msgid_ros = -1;

void send_mode_to_unitree_sdk(unsigned char mode)
{
        ros_msg send_msg = {0};
        if (msgid_ros == -1)
        {
               RCLCPP_INFO(rclcpp::get_logger("msgid_ros"), "msgid_ros [%d]", msgid_ros);
                return;
        }
        send_msg.msg_type = CMD_SET_DOG_MODE;
        send_msg.set_mode.setMode   = mode;
        RCLCPP_INFO(rclcpp::get_logger("setMode"), "I heard: [%d]", send_msg.set_mode.setMode);
        msg_queue_send(msgid_ros, &send_msg, sizeof(send_msg));
}

void setModeCallback(const a1_msgs::msg::CmdMode::SharedPtr msg)
{
    RCLCPP_INFO(rclcpp::get_logger("rcv_mode"), "I heard: [%d]", msg->mode);
    if (msg->mode > 3){
        RCLCPP_INFO(rclcpp::get_logger("rcv_mode"), "set mode invalid: %d", msg->mode);
         return;
    }
    send_mode_to_unitree_sdk(msg->mode);
}
void ros2_cmdMode_subscription(int argc, char* argv[])
{
    std::cout << "setMode_subscription" << std::endl;
    rclcpp::init(argc, argv);
	
    auto node = std::make_shared<rclcpp::Node>("cmd_mode");
    rclcpp::Subscription<a1_msgs::msg::CmdMode>::SharedPtr mode_sub;
    mode_sub = node->create_subscription<a1_msgs::msg::CmdMode>(
                "cmd_mode",
                10,
                setModeCallback);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return;
}
void send_vel_to_unitree_sdk(double forwardSpeed, double sideSpeed, double rotateSpeed)
{
	ros_msg send_msg = {0};
	if (msgid_ros == -1)
	{
	       RCLCPP_INFO(rclcpp::get_logger("msgid_ros"), "msgid_ros [%d]", msgid_ros);
		return;
	}
	send_msg.msg_type      = CMD_SET_DOG_VEL;
	send_msg.set_vel.forwardSpeed = forwardSpeed;
	send_msg.set_vel.sideSpeed      = sideSpeed;
	send_msg.set_vel.rotateSpeed   = rotateSpeed;
	//RCLCPP_INFO(rclcpp::get_logger("setMode"), "");
	
	msg_queue_send(msgid_ros, &send_msg, sizeof(send_msg));
}

void setVelCallback(const a1_msgs::msg::CmdVel::SharedPtr msg)
{
    RCLCPP_INFO(rclcpp::get_logger("rcv_vel"), "I heard: forwardspeed[%0.2f],sidespeed[%0.2f] rotatespeed[%0.2f]", msg->forwardspeed,msg->sidespeed,msg->rotatespeed);
    send_vel_to_unitree_sdk(msg->forwardspeed, msg->sidespeed, msg->rotatespeed);
}
void ros2_cmdVel_subscription(int argc, char* argv[])
{
    std::cout << "setMode_subscription" << std::endl;
    rclcpp::init(argc, argv);
	
    auto node = std::make_shared<rclcpp::Node>("cmd_vel");
    rclcpp::Subscription<a1_msgs::msg::CmdVel>::SharedPtr mode_sub;
    mode_sub = node->create_subscription<a1_msgs::msg::CmdVel>(
                "cmd_vel",
                10,
                setVelCallback);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return;
}
int main(int argc, char* argv[])
{
    msgid_ros = create_msg_queue(0);
    if (strstr(argv[0], "sub_mode") != NULL)
          ros2_cmdMode_subscription(argc, argv);
    else if (strstr(argv[0], "sub_vel") != NULL)
    	   ros2_cmdVel_subscription(argc, argv);
    return 0;
}
