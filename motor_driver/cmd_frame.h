#ifndef CMD_FRAME_H
#define CMD_FRAME_H

#ifndef BTYE
#define BYTE unsigned char
#endif

//frame head
#define FRAME_HEAD_1    0x55
#define FRAME_HEAD_2    0xAA
//frame end
#define FRAME_END       0x0A

//frame type
typedef enum{
  MSG_ODOM = 1,
  MSG_SPEED = 2,
  MSG_BATTERY = 3,
  MSG_ACCE  = 4,
  MSG_WHEEL  = 5,
  MSG_MOVECTRL = 0x81,
  MSG_SERVOCTRL = 0x82
}msg_type;

//MASTER_CMD
#define MASTER_DATA_LENGTH		12
#define MASTER_XOR_FLAG			MASTER_DATA_LENGTH+3
#define MASTER_END_FLAG			MASTER_XOR_FLAG+1
#define MASTER_FRAME_LENGTH		MASTER_END_FLAG+1

//ROBOT_MSG
#define ROBOT_DATA_LENGTH		8
#define ROBOT_XOR_FLAG			ROBOT_DATA_LENGTH+3
#define ROBOT_END_FLAG			ROBOT_XOR_FLAG+1
#define ROBOT_FRAME_LENGTH		ROBOT_END_FLAG+1

//#define PI 3.14159

typedef struct
{
  long counter;
  int count;
  char dir;
  char encoder_a;
  char encoder_b;
}Wheel;

typedef struct
{
  long counter1;
  long counter2;
  long counter3;
}SRobotWheelData;

typedef struct
{

    int odom_motor1;
    int odom_motor2;
    int odom_motor3;

}SRobotOdomData;

typedef struct
{

    float v_motor1;
    float v_motor2;
    float v_motor3;

}SRobotSpeedData;

typedef struct
{

    float v_motor1;
    float v_motor2;
    float v_motor3;

}RRobotData;

typedef struct
{

float voltage;
float current;
float charge;

}SRobotBatteryData;

typedef struct
{

float acx;
float acy;
float acz;

}SRobotAcceData;

typedef struct
{

float gyx;
float gyy;
float gyz;

}SRobotGyroData;

typedef struct
{

float yaw;
float pitch;
float roll;

}SRobotYPRData;


#endif
