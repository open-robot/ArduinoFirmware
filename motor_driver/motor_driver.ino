/****************************************************************************

Copyright (c) 2016, <Di Zhu, Lisa Shi>, Intel Corporation.

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License version 2 as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program (see the file COPYING); if not, write to the
Free Software Foundation, Inc.,
51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA

****************************************************************************/

#include <math.h>
#include <DueTimer.h>
#include <string.h>
#include "cmd_frame.h"  //the serial_communication library
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

/****************************************************************************/
//#define USE_MPU 1

/*********************************choose motor*******************************/
#define MOTOR_370_333RPM

#ifdef MOTOR_370_333RPM
#define KP  3.6
#define KI  0.1
#define KD  0.0005
#define ROTATE_SPEED  6.0606
#endif
/******************************************************************************/

//the pin num of motor, two pins for 1 motor,  MOTOR1_A and MOTOR1_B are for the motor1
#define MOTOR1_A  2
#define MOTOR1_B  3
#define MOTOR2_A  6
#define MOTOR2_B  7
#define MOTOR3_A  8
#define MOTOR3_B  9                                                                                                              

//the pin number of encoder, ENCODERX is for motorX
#define ENCODER1_A  22
#define ENCODER1_B  24
#define ENCODER2_A  26
#define ENCODER2_B  28
#define ENCODER3_A  30
#define ENCODER3_B  32


#define BUADRATE 576000
#define intervalTime_timer8  30000  //30ms  intervalTime_timer8 should be larger than 100ms without watchdog
#define amount_pulse 5    //arduino calculates the speed of a wheel every "amount_pulse"  pulses
#define dis_pwm  30   //the difference of each neighbor pwm is not bigger than 30 : the limitation of acceleration

#define  OUTPUT_READABLE_GYRO
#define  OUTPUT_READABLE_YAWPITCHROLL

// SERVO to control through ROS
//#define SERVO_PIN 4
#ifdef SERVO_PIN
#include <Servo.h>
Servo myservo;  // create servo object to control a servo
#endif

#define DEBUG
#ifdef DEBUG
#define debug_print(x) Serial.print(x)
#define debug_println(x) Serial.println(x)
#define debug_print_hex(x) Serial.print(x, HEX)
#else
#define debug_print(x)
#define debug_println(x)
#define debug_print_hex(x)
#endif

#ifndef _WATCHDOG_
#define _WATCHDOG_
#endif


#ifdef USE_MPU
#define MPU_INT  36
MPU6050 mpu;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
const int MPU_addr = 0x68; // I2C address of the MPU-6050
int16_t mpu6050[6] = {0}; //AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
boolean mpu_ready = false;

#endif


// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int16_t gyro[3];
int16_t acc[3];

long count_timer1 = 0; // number of timer2() called
long count_timer2 = 0; // number of timer2() called
float ang[3] = {0};   //the speed of each wheel,r/min

//Setpoint_X is the Target value of PID controller, Input_X is the speed of wheel, Output_X is pwm
double Setpoint_1 = 50, Input_1 = 0, Output_1 = 0;
double Setpoint_2 = 50, Input_2 = 0, Output_2 = 0;
double Setpoint_3 = 50, Input_3 = 0, Output_3 = 0;
double Output_1_last = 0, Output_2_last = 0, Output_3_last = 0;

//the parameter of PID controller
SRobotOdomData odomData;
SRobotSpeedData speedData;
SRobotBatteryData batteryData;
SRobotAcceData acceData;
SRobotGyroData gyroData;
SRobotYPRData yprData;

unsigned char buf = 0;
unsigned long watch_dog = 0, last_time = 0;
RRobotData speed_data; //the structure of receiving data from Minnow or nuc
RRobotData speed_motor;
char speed_buffer[20], speed_frame[20];
boolean stringComplete = false;  // whether the string is complete
int serial_cnt = 0; //global


bool flagBattery = false;
bool flagOdomSpeed = false;
bool flagDmpRead = false;
bool flagDmpWrite = false;
bool flagInit = false;
int timer_timer = 0;
float offsetYaw;
float offsetPitch;
float offsetRoll;
Wheel omni_wheel[3];
bool pidflag=false;
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void Software_Reset() {

#ifdef USE_MPU
    mpu.reset();
#endif

    const int RSTC_KEY = 0xA5;
    RSTC->RSTC_CR = RSTC_CR_KEY(RSTC_KEY) | RSTC_CR_PROCRST | RSTC_CR_PERRST;
    while (true);
}

void Count_and_Direction(Wheel *omni) {
    omni->count += 1;
    if (digitalRead(omni->encoder_a) == HIGH) {
        if (digitalRead(omni->encoder_b) == LOW) {
            omni->dir = 0; //forward
            omni->counter ++;
        }
        else {
            omni->dir = 1; //backward
            omni->counter -= 1;
        }
    }
    else if (digitalRead(omni->encoder_b) == HIGH) {
        omni->dir = 0; //forward
        omni->counter ++;
    }
    else {
        omni->dir = 1; //backward
        omni->counter -= 1;
    }
// Serial.println(omni->count);
}

//interrupt function of encoder1
void Encoder_Counter_1() {
    Count_and_Direction(&omni_wheel[0]);
}

//interrupt function of encoder2
void Encoder_Counter_2() {
    Count_and_Direction(&omni_wheel[1]);
}

//interrupt function of encoder3
void Encoder_Counter_3() {
    Count_and_Direction(&omni_wheel[2]);
}

int match_flag = 0;
BYTE inChar;
BYTE match[2];
void serialEvent1() {
    if (Serial1.available()) {
        if (match_flag == 0) {
            match[0] = (unsigned char)Serial1.read();
            if (match[0] == 0x55) {
                match_flag = 1;
                speed_buffer[0] = 0x55;
                serial_cnt ++;
                return;
            }
            else {
            match_flag = 0;
            serial_cnt = 0;
            return;
            }
        }
        if (match_flag == 1) {
            match[1] = (unsigned char)Serial1.read();
            if (match[1] == 0xAA) {
                match_flag = 2;
                speed_buffer[1] = 0xAA;
                serial_cnt ++;
                return;
            }
            else {
                match_flag = 0;
                serial_cnt = 0;
                return;
            }
        }
        if (match_flag == 2) {
            inChar = (unsigned char)Serial1.read();
            speed_buffer[serial_cnt] = inChar;
            serial_cnt ++;
            if (serial_cnt >= 20) {
                stringComplete = true;
                serial_cnt = 0;
                match_flag = 0;
                watch_dog = millis();
                return;
            }
        }
    }
}


float velocity_calculate(Wheel *omni) {
    //calculating the speed of wheel_1, ang[X]:r/min
    float ang;
    //ang = float(250.00 * (count) / float(5.50 * float(intervalTime_timer3 / 1000.00)));
    //formula:ang = (count/1320)*1000*60/(1320*20));
    ang = float(omni->count *ROTATE_SPEED);
    if (omni->dir== 1) {
        ang = -ang;
    }
    omni->count = 0;
    return ang;
}

#define SAMPLE_LEN 60
int get_current() {
    uint32_t sum = 0;
    for (int i = 0; i < SAMPLE_LEN; i ++) {
    sum += analogRead(A0);
    delayMicroseconds(30);
    }
    return (2490 - map(sum / SAMPLE_LEN, 0, 4096, 0, 3300)) / 0.185;
}

int get_voltage() {
    int sensor_value;
    sensor_value = map(analogRead(A1), 0, 4096, 0, 3300);
    if (sensor_value ) {
        // 12.6v  R1 4.02k , R2 12.1k
        return map(sensor_value - 2740, 0, 560, 1100, 1320);
    }
}

void pack_message_and_send(void* data, msg_type type, BYTE sub_type=0){
    BYTE a = 0;
    BYTE buffer[20];
    buffer[0] = FRAME_HEAD_1;
    buffer[1] = FRAME_HEAD_2;
    buffer[2] = sub_type;
    buffer[3] = type;
    memcpy(&buffer[4], data, 12);
    for (int i = 3; i < 16; i++) {
        a += buffer[i];
    }
    buffer[16] = a;
    a = 0;
    buffer[17] = 0;
    buffer[18] = 0;
    buffer[19] = FRAME_END;

    #ifdef DEBUG
        for (int i=0; i < 20; i ++){
            debug_print_hex(buffer[i]);
            debug_print(" ");
        }
        debug_println("");
        Serial1.write(buffer, 20);
    #else
        Serial1.write(buffer, 20);
    #endif

}

void timer8() {
    timer_timer++;
    if (timer_timer >= 4) {
        count_timer2 ++;
        if (count_timer2 > 100) {
            flagBattery = true;
            count_timer2 = 0;
            flagOdomSpeed = true;
            flagDmpWrite = true;
        }
        else {
            flagOdomSpeed = true;
            flagDmpWrite = true;
        }
        timer_timer = 0;
    }
    ang[0] = velocity_calculate(&omni_wheel[0]);
    ang[1] = velocity_calculate(&omni_wheel[1]);
    ang[2] = velocity_calculate(&omni_wheel[2]);
    /*Serial.print("ang[]");
    Serial.println(ang[0]);
    Serial.println(ang[1]);
    Serial.println(ang[2]);*/
    omni_wheel[0].count = 0;
    omni_wheel[1].count = 0;
    omni_wheel[2].count = 0;
    if(pidflag) {
        PID_controller(speed_motor.v_motor1, &Input_1, &Setpoint_1, &Output_1, &Output_1_last,  MOTOR1_A, MOTOR1_B, &ang[0]);
        PID_controller(speed_motor.v_motor2, &Input_2, &Setpoint_2, &Output_2, &Output_2_last,  MOTOR2_A, MOTOR2_B, &ang[1]);
        PID_controller(speed_motor.v_motor3, &Input_3, &Setpoint_3, &Output_3, &Output_3_last,  MOTOR3_A, MOTOR3_B, &ang[2]);
    }
    /* Serial.print("setpoint");
    Serial.println(speed_motor.v_motor1);
    Serial.println(speed_motor.v_motor2);
    Serial.println(speed_motor.v_motor3);*/
}

double interg;
double lastInput;
void PID_controller(float v_motor, double * Input, double * Setpoint, double * Output, double * Output_last, int MotorPin_A, int MotorPin_B, float * ang) {
    if (abs(v_motor) <= 300 && abs(v_motor) >= 5) {
        double input_error=0;
        double diff_input=0;
        *Input = abs(*ang);   
        *Setpoint = abs(v_motor);
        input_error= *Setpoint-*Input;
        interg +=(KI *input_error);
        if(interg >200)
            interg=200;
        else if(interg <0)
            interg =0;
        diff_input = (*Input - lastInput);
        *Output=KP*input_error+interg-KD*diff_input;
        lastInput=*Input;
        if (*Output - *Output_last > dis_pwm) {
            *Output = *Output_last + dis_pwm;
        }
        //!!!!!!!!!if the *Output is larger than 300, the PID controller will be blocked!!!!!
        if (*Output > 200) {
            *Output = 200;
        }
        *Output_last = *Output;
        //  debug_println("output");
        //  debug_println(*Output);
        if (v_motor >= 0) {
            analogWrite(MotorPin_B, LOW);
            analogWrite(MotorPin_A, *Output);
        }
        else {
            digitalWrite(MotorPin_A, LOW);
            analogWrite(MotorPin_B, *Output);
        }
    }
    else {
    analogWrite(MotorPin_A, HIGH);
    analogWrite(MotorPin_B, HIGH);
    *ang = 0;
    }
}

#ifdef USE_MPU
void mpuInit(){
    mpu.resetI2CMaster();
    mpu.reset();
    // initialize device
    //debug_println(F("Initializing I2C devices..."));
    mpu.initialize();
    // verify connection
    debug_println(F("Testing device connections..."));
    debug_println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    if (mpu.testConnection()){
        mpu_ready = true;
    }
    /*
    // wait for ready
    debug_println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
    */
    // load and configure the DMP
    //debug_println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(32);
    mpu.setYGyroOffset(14);
    mpu.setZGyroOffset(62);
    mpu.setXAccelOffset(-793);
    mpu.setYAccelOffset(1138);
    mpu.setZAccelOffset(1368);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        //debug_println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        // enable Arduino interrupt detection
        //debug_println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //debug_println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } 
    else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        debug_print(F("DMP Initialization failed (code "));
        debug_print(devStatus);
        debug_println(F(")"));
    }
}

void dmpDataReady() {
    flagDmpRead = true;
    /*
    //////////////////////////////////////////////////////////////
    //////////////////////////DMP//////////////////////////////
    /////////////////////////////////////////////////////////////
    debug_println("DmpRead");
    mpuIntStatus = mpu.getIntStatus();
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        //debug_println(F("FIFO overflow!"));
        // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        //mpu.resetFIFO();
        //flagDmpRead = false;
    }
    */
}
#endif
void setup() {
    // Debug toggle

    #ifdef DEBUG
        Serial.begin(115200);
        debug_println("restart...");
    #endif

    // Motor pins init
    pinMode(MOTOR1_A, OUTPUT);
    pinMode(MOTOR1_B, OUTPUT);
    digitalWrite(MOTOR1_A, LOW);
    digitalWrite(MOTOR1_B, LOW);

    pinMode(MOTOR2_A, OUTPUT);
    pinMode(MOTOR2_B, OUTPUT);
    digitalWrite(MOTOR2_A, LOW);
    digitalWrite(MOTOR2_B, LOW);

    pinMode(MOTOR3_A, OUTPUT);
    pinMode(MOTOR3_B, OUTPUT);
    digitalWrite(MOTOR3_A, LOW);
    digitalWrite(MOTOR3_B, LOW);

    // Encoder pins init
    pinMode(ENCODER1_A, INPUT_PULLUP);
    pinMode(ENCODER1_B, INPUT_PULLUP);
    pinMode(ENCODER2_A, INPUT_PULLUP);
    pinMode(ENCODER2_B, INPUT_PULLUP);
    pinMode(ENCODER3_A, INPUT_PULLUP);
    pinMode(ENCODER3_B, INPUT_PULLUP);

    omni_wheel[0].encoder_a = ENCODER1_A;
    omni_wheel[0].encoder_b = ENCODER1_B;
    omni_wheel[0].counter = 0;
    omni_wheel[0].count=0;
    omni_wheel[0].dir = 0;

    omni_wheel[1].encoder_a = ENCODER2_A;
    omni_wheel[1].encoder_b = ENCODER2_B;
    omni_wheel[1].counter = 0;
    omni_wheel[1].count=0;
    omni_wheel[1].dir = 0;

    omni_wheel[2].encoder_a = ENCODER3_A;
    omni_wheel[2].encoder_b = ENCODER3_B;
    omni_wheel[2].counter = 0;
    omni_wheel[2].count=0;
    omni_wheel[2].dir = 0;

    // Attach int
    attachInterrupt(digitalPinToInterrupt(ENCODER1_A), Encoder_Counter_1, FALLING);
    attachInterrupt(digitalPinToInterrupt(ENCODER2_A), Encoder_Counter_2, FALLING);
    attachInterrupt(digitalPinToInterrupt(ENCODER3_A), Encoder_Counter_3, FALLING);

    //attachInterrupt(ENCODER1_B, Encoder_Counter_4, FALLING);
    //attachInterrupt(ENCODER2_B, Encoder_Counter_5, FALLING);
    //attachInterrupt(ENCODER3_B, Encoder_Counter_6, FALLING);

    //Timer1.attachInterrupt(timer1).start(intervalTime_timer1);
    Timer8.attachInterrupt(timer8).start(intervalTime_timer8); //calculating the speed every 10ms

    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    //#ifndef DEBUG
        Serial1.begin(BUADRATE);
        delay(500);
        match_flag = 0;
    //#endif

#ifdef USE_MPU
    mpuInit();
    // MPU INT init
    pinMode(MPU_INT, INPUT);
    attachInterrupt(digitalPinToInterrupt(MPU_INT), dmpDataReady, RISING);
#endif
    speed_motor.v_motor1 = 0;
    speed_motor.v_motor2 = 0;
    speed_motor.v_motor3 = 0;
    // Set ADC to 12 bits
    analogReadResolution(12);
    #ifdef _WATCHDOG_
        //Enable watchdog
        watchdogEnable(100);
    #endif
    // Servo channel init
    #ifdef SERVO_PIN
        myservo.attach(SERVO_PIN);
    #endif

}

#ifdef _WATCHDOG_
    // this function has to be present here
    void watchdogSetup(void)
    {

    }
#endif

void loop() {

    //if no data comes in, stop
    int k = 0;
    BYTE aa = 0;

    //#ifdef _WATCHDOG_
    //  Watchdog reset
    watchdogReset();
    //#endif
    if (stringComplete) {
        aa = 0;
        for (k = 0; k < 20; k++) {
            speed_frame[k] = speed_buffer[k];
        }
        if (speed_frame[3] == 0x05 && speed_frame[19] == 0x0A) {
            Software_Reset();
        }
        else if (speed_frame[3] == MSG_MOVECTRL && speed_frame[19] == 0x0A) {
            for (int i = 3; i < 16; i++) {
                aa += speed_frame[i];
            }
            if (aa == speed_frame[16]) {
                memcpy(&(speed_data.v_motor1), &speed_frame[4], 4);
                memcpy(&(speed_data.v_motor2), &speed_frame[8], 4);
                memcpy(&(speed_data.v_motor3), &speed_frame[12], 4);
                aa = 0;
                speed_motor.v_motor1 = speed_data.v_motor1;
                speed_motor.v_motor2 = speed_data.v_motor2;
                speed_motor.v_motor3 = speed_data.v_motor3;
            }
            else {
                debug_println("check_sum error!");
                aa = 0;
            }
        #ifdef SERVO_PIN
        }
        else if (speed_frame[3] == MSG_SERVOCTRL && speed_frame[19] == 0x0A) { // PWM channel control message
        myservo.write(speed_frame[4]);
        #endif
        }
    stringComplete = false;
    }
    last_time = millis() - watch_dog;
    //当上位机停止发送数据或其程序未启动时，保证电机不转
    //上位机串口发送的帧间隔时间要小于1300ms，否则电机不响应或转动时断时续
    if (last_time <= 1300) {
        pidflag=true;
    }
    else {
        analogWrite(MOTOR1_A, HIGH);
        analogWrite(MOTOR1_B, HIGH);
        analogWrite(MOTOR2_A, HIGH);
        analogWrite(MOTOR2_B, HIGH);
        analogWrite(MOTOR3_A, HIGH);
        analogWrite(MOTOR3_B, HIGH);
        speed_motor.v_motor1 = 0;
        speed_motor.v_motor2 = 0;
        speed_motor.v_motor3 = 0;
    }
    if (flagBattery) {
        batteryData.voltage = get_voltage();
        batteryData.current = get_current();
        batteryData.charge = 0;
        pack_message_and_send(&batteryData, MSG_BATTERY);
        flagBattery = false;
    }


    if (flagOdomSpeed) {
        odomData.odom_motor1 = omni_wheel[0].counter;
        odomData.odom_motor2 = omni_wheel[1].counter;
        odomData.odom_motor3 = omni_wheel[2].counter;
        pack_message_and_send(&odomData, MSG_ODOM);

        speedData.v_motor1 = ang[0];
        speedData.v_motor2 = ang[1];
        speedData.v_motor3 = ang[2];
        pack_message_and_send(&speedData, MSG_SPEED);

        flagOdomSpeed = false;
    }


#ifdef USE_MPU
   
    if ( mpu_ready && flagDmpRead) {
        //debug_println("DmpRead");
        mpuIntStatus = mpu.getIntStatus();
        // get current FIFO count
        fifoCount = mpu.getFIFOCount();
        // check for overflow (this should never happen unless our code is too inefficient)
        if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
            // reset so we can continue cleanly
            mpu.resetFIFO();
            //debug_println(F("FIFO overflow!"));
            // otherwise, check for DMP data ready interrupt (this should happen frequently)
        }
        else if (mpuIntStatus & 0x02) {
            // wait for correct available data length, should be a VERY short wait
            while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
            // read a packet from FIFO
            mpu.getFIFOBytes(fifoBuffer, packetSize);
            // track FIFO count here in case there is > 1 packet available
            // (this lets us immediately read more without waiting for an interrupt)
            fifoCount -= packetSize;
            //mpu.resetFIFO();
            flagDmpRead = false;
        }
    }

    //  debug_print("3:");
    //  debug_println(millis());

    if (mpu_ready && flagDmpWrite && millis() > 10000)  {
        //debug_println("DmpWrite");
        #ifdef OUTPUT_READABLE_GYRO
            mpu.getMotion6(&acc[0], &acc[1], &acc[2], &gyro[0], &gyro[1], &gyro[2]);
            gyroData.gyx = float(gyro[0]) * 0.061 * M_PI / 180;
            gyroData.gyy = float(gyro[1]) * 0.061 * M_PI / 180;
            gyroData.gyz = float(gyro[2]) * 0.061 * M_PI / 180;
            pack_message_and_send(&gyroData, MSG_ACCE, 1);
        
            //debug_println("gyroData: ");
            //debug_print(gyroData.gyx);
            //debug_print(' ');
            //debug_print(gyroData.gyy);
            //debug_print(' ');
            //debug_println(gyroData.gyz);
       
        /*
            mpu.dmpGetGyro(gyro, fifoBuffer);
            gyroData.gyx = float(gyro[0]) * 0.061 * 25;
            gyroData.gyy = float(gyro[1]) * 0.061 * 25;
            gyroData.gyz = float(gyro[2]) * 0.061 * 25;
            debug_println("dmpGyrooData: ");
            debug_print(gyroData.gyx);
            debug_print(' ');
            debug_print(gyroData.gyy);
            debug_print(' ');
            debug_println(gyroData.gyz);*/

        #endif

    #ifdef OUTPUT_READABLE_YAWPITCHROLL
        if (!flagInit) {
            flagInit = true;
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            yprData.yaw = ypr[0];
            offsetYaw = yprData.yaw;
            //debug_println("offset:");
            //debug_println(offsetYaw);
        }
        else {
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

            yprData.yaw = ypr[0];
            yprData.pitch = ypr[1];
            yprData.roll = ypr[2];
            if (yprData.yaw >= offsetYaw && yprData.yaw <= M_PI) {
                yprData.yaw = -yprData.yaw + 2 * M_PI + offsetYaw;
            }
            else {
                yprData.yaw = -yprData.yaw + offsetYaw;
            }
            pack_message_and_send(&yprData, MSG_ACCE, 2);

            //debug_print("ypr\t");
            //debug_println(yprData.yaw);
            //debug_print("\t");
            //debug_print(yprData.pitch);
            //debug_print("\t");
            //debug_println(yprData.roll);
        }
    #endif
    flagDmpWrite = false;
    }
#endif
}

