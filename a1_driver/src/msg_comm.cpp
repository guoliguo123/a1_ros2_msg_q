#include "driver.h"
#include "rclcpp/rclcpp.hpp"

int create_msg_queue(int flag)
{
	key_t key = ftok(".", 111);
	if (key == -1)
	{
	       std::cout << "ftok get fail " << std::endl;
		return -1;
	}
	std::cout << "ftok get success (ID=)" << key << std::endl;
	int msgid = msgget(key, flag);
	if (msgid == -1)
	{
	       
       	return -1;
	}
	std::cout << "create message success" << std::endl;
	return msgid;
}
int msg_queue_send(int msgid, ros_msg *send_msg, int len)
{
	//int  msgsnd ( int msgid ,  struct msgbuf*msgp , int msgsz, int msgflg );
	int res = msgsnd(msgid, send_msg, len, 0);
	if (res == -1)
	{
		RCLCPP_INFO(rclcpp::get_logger("msgsnd"), "msgsend fail");
		return -1;
	}
	return 0;
}

int msg_queue_rcv(int msgid, ros_msg * rcv_msg)
{
	RCLCPP_INFO(rclcpp::get_logger("rcv msg"), "sizeof(ros_msg %d)",sizeof(ros_msg));
	//int  msgrcv( int  msgid , struct   msgbuf*  msgp ,  int msgsz ,  long msgtyp, int msgflg);
	int res = msgrcv(msgid, rcv_msg, sizeof(ros_msg), 0, 0);
	if (res == -1)
	{
		RCLCPP_INFO(rclcpp::get_logger("rcv msg fail"), "return error code -1");
		return -1;
	}
	RCLCPP_INFO(rclcpp::get_logger("rcv setMode"), "msg_type [%d]", rcv_msg->msg_type);
	return 0;
}
