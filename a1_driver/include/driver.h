#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
using namespace UNITREE_LEGGED_SDK;

class A1Wrapper{
public:
    A1Wrapper(uint8_t level): safe(LeggedType::A1), udp(level){
            udp.InitCmdData(cmd);
    }
    //friend void setModeCallback(const CmdMode::SharedPtr msg);
    void UDPRecv();
    void UDPSend();
    void RobotControl();
    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};
    int motiontime = 0;
    float dt = 0.002;     // 0.001~0.01
private:
    void robotToLocalTime();
    void walkCmd(float forwardSpeed, float sideSpeed, float rotateSpeed, float time);
    void standCmd();
};

enum {
	CMD_SET_DOG_MODE =1,
       CMD_SET_DOG_VEL
};
typedef struct msg_q{
	long msg_type;
	union {
       	unsigned char setMode;
	}set_mode;
	union {
		float forwardSpeed;
		float sideSpeed;
		float rotateSpeed;
	}set_vel;
}ros_msg;

int msg_queue_rcv(int msgid, ros_msg * rcv_msg);
int create_msg_queue(int flag);
int msg_queue_send(int msgid, ros_msg *buf, int len);
