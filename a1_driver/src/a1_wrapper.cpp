#include "driver.h"
#include "rclcpp/rclcpp.hpp"

static int msgid_wrapper = -1;
void A1Wrapper::walkCmd(float forwardSpeed, float sideSpeed, float rotateSpeed, float time) {
//    build udp command


}
//0:stand  1:force stand 2:walk
void A1Wrapper::RobotControl()
{     
	int ret;
       ros_msg rcv_msg = {0};
	int msg_type = 0;
       if (msgid_wrapper != -1)
       {
           ret = msg_queue_rcv(msgid_wrapper, &rcv_msg);
	    if (ret == -1)
	    {
  	         
	    }
	    msg_type = rcv_msg.msg_type;
           switch (msg_type){
			case CMD_SET_DOG_MODE:
				cmd.mode = rcv_msg.set_mode.setMode;
				ret = udp.SetSend(cmd);
				if (ret != 0)
				{
 					RCLCPP_INFO(rclcpp::get_logger("SetSend"), "error code [%d]", ret);
				}
				RCLCPP_INFO(rclcpp::get_logger("SetSend"), "SetSend Success [%d]", cmd.mode);
				break;
			case CMD_SET_DOG_VEL:
				RCLCPP_INFO(rclcpp::get_logger("SetVel"), "forwardSpeed(%0.2f) sideSpeed(%0.2f) rotateSpeed(%0.2f)!!!",rcv_msg.set_vel.forwardSpeed,
					rcv_msg.set_vel.sideSpeed, rcv_msg.set_vel.rotateSpeed);
				cmd.forwardSpeed  = rcv_msg.set_vel.forwardSpeed;
                           cmd.sideSpeed       = rcv_msg.set_vel.sideSpeed;
                           cmd.rotateSpeed    = rcv_msg.set_vel.rotateSpeed;
			       ret = udp.SetSend(cmd);
				if (ret != 0)
				{
 					RCLCPP_INFO(rclcpp::get_logger("SetVel"), "error code [%d]", ret);
				}
				
				RCLCPP_INFO(rclcpp::get_logger("SetVel"), "SetSend Success !!!");
				break;
			default:
				break;
		   }
       }
}
void A1Wrapper::UDPRecv()
{
    udp.Recv();
}

void A1Wrapper::UDPSend()
{  
    udp.Send();
}

int main(void) 
{
    std::cout << "Communication level is set to HIGH-level." << std::endl
              << "WARNING: Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    A1Wrapper wrapper(UNITREE_LEGGED_SDK::HIGHLEVEL);
    UNITREE_LEGGED_SDK::InitEnvironment();
    UNITREE_LEGGED_SDK::LoopFunc loop_control("control_loop", wrapper.dt,  boost::bind(&A1Wrapper::RobotControl, &wrapper));
    UNITREE_LEGGED_SDK::LoopFunc loop_udpSend("udp_send",  wrapper.dt, 1, boost::bind(&A1Wrapper::UDPSend, &wrapper));
    UNITREE_LEGGED_SDK::LoopFunc loop_udpRecv("udp_recv",  wrapper.dt, 1, boost::bind(&A1Wrapper::UDPRecv, &wrapper));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();
    //create msg queue
    msgid_wrapper = create_msg_queue(IPC_CREAT|0777);
    std::cout << "msgid = " << msgid_wrapper << std::endl;
    while(1){
        sleep(10);
    };

    return 0; 
}
