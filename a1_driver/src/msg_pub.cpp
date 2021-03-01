#include <iostream>
#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "a1_msgs/msg/cmd_mode.hpp"
#include "a1_msgs/msg/cmd_vel.hpp"
#include "driver.h"

using namespace std::chrono_literals;

class PubNode: public rclcpp::Node
{
public:
    PubNode(uint8_t setflag, uint8_t mode, float forwardspeed = 0, 
		float rotatespeed = 0, float sidespeed = 0):Node("set_mode"), count_(0){
        m_setflag = setflag;
	 if (setflag == CMD_SET_DOG_MODE) 
	 {
	     m_mode = mode;
	     mode_pub = this->create_publisher<a1_msgs::msg::CmdMode>("cmd_mode", 10);
	     
	 }
	 else if (setflag == CMD_SET_DOG_VEL)
	 {
		m_forwardspeed = forwardspeed;
		m_rotatespeed   = rotatespeed;
		m_sidespeed      = sidespeed;
		vel_pub = this->create_publisher<a1_msgs::msg::CmdVel>("cmd_vel", 10);
	 }
        //mode_pub = this->create_publisher<CmdMode>("cmd_mode", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&PubNode::pub_callback, this));
    }
private:
    void pub_callback(){
        
        if (m_setflag == CMD_SET_DOG_MODE) 
	 {
	     a1_msgs::msg::CmdMode msg;
	     msg.mode = m_mode;
	     RCLCPP_INFO(this->get_logger(), "sending mode %d",msg.mode);
            mode_pub->publish(msg);
	 }
	 else if (m_setflag == CMD_SET_DOG_VEL)
	 {
	       a1_msgs::msg::CmdVel msg;
		msg.forwardspeed = m_forwardspeed;
		msg.rotatespeed    = m_rotatespeed;
		msg.sidespeed       = m_sidespeed;
		RCLCPP_INFO(this->get_logger(), "forwardspeed (%0.2f) rotatespeed(%0.2f) sidespeed(%0.2f)",msg.forwardspeed,
			msg.rotatespeed,msg.sidespeed);
              vel_pub->publish(msg);
	 }
        
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<a1_msgs::msg::CmdMode>::SharedPtr mode_pub;
    rclcpp::Publisher<a1_msgs::msg::CmdVel>::SharedPtr vel_pub;
    size_t count_;
    uint8_t m_mode;
    uint8_t m_setflag;
    float m_forwardspeed;
    float m_rotatespeed;
    float m_sidespeed;
};

void ros2_cmdMode_publish(int argc, char* argv[])
{
    unsigned char mode;
    std::cout << "pub start" << std::endl;
    std::cout << argv[0] <<std::endl;
    rclcpp::init(argc, argv);
    mode = uint8_t(atoi(argv[1]));
    auto node = std::make_shared<PubNode>(CMD_SET_DOG_MODE, mode);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return;
} 

void ros2_cmdVel_publish(int argc, char* argv[])
{
    float forwardspeed = 0;
    float rotatespeed = 0;
    float sidespeed = 0;
    std::cout << "pub set Vel start" << std::endl;
    std::cout << argv[0] <<std::endl;
    rclcpp::init(argc, argv);
    if (argv[1] == NULL || argv[2] == NULL || argv[3] == NULL)
    {
	 std::cout << "vel is NULL" << std::endl;
        return;
    }
    forwardspeed = float(atof(argv[1]));
    rotatespeed   = float(atof(argv[2]));
    sidespeed      = float(atof(argv[3]));
    auto node = std::make_shared<PubNode>(CMD_SET_DOG_VEL, 0, forwardspeed, rotatespeed, sidespeed);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return;
} 
int main(int argc, char * argv[])
{
    if (strstr(argv[0], "pub_mode") != NULL)
        ros2_cmdMode_publish(argc, argv);
    else if (strstr(argv[0], "pub_vel") != NULL)
	 ros2_cmdVel_publish(argc, argv);
    return 0;

}
