
//As a base for this code I'm using one of the examples provided by ROBOTIS CO., LTD. 
//for the use of dynamixels here. So this part stays.

// Copyright 2020 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <ros/ros.h>

#include "std_msgs/String.h"
#include "dynamixel_sdk_examples/GetPosition.h"
#include "dynamixel_sdk_examples/SetPosition.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

#include "sensor_msgs/Joy.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/Twist.h"

using namespace dynamixel;

// Control table address
//https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/     <-- for the dynamixels im using
#define ADDR_TORQUE_ENABLE    64
#define ADDR_GOAL_POSITION    116
#define ADDR_PRESENT_POSITION 132

#define ADDR_GOAL_VELOCITY    104

#define ADDR_OPERATING_MODE   11

#define ADDR_POS_PID_P        84            //default value 640
#define ADDR_POS_PID_I        82            //default value 0
#define ADDR_POS_PID_D        80            //default value 3600

#define ADDR_VEL_PI_P         78            //default value 100
#define ADDR_VEL_PI_I         76            //default value 1000


// Protocol version
#define PROTOCOL_VERSION      2.0             // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define DXL1_ID               1               // DXL1 ID
#define DXL2_ID               2               // DXL2 ID
#define DXL3_ID               3               // DXL3 ID

#define BAUDRATE              1000000         // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME           "/dev/ttyUSB0"  // [Linux] To find assigned port, use "$ ls /dev/ttyUSB*" command

//State LED
#define REDLED                10
#define GREENLED              11
#define BLUELED               12

//Flags
bool a_button_pressed = false;

int current_mode = 0;         // 0 - manual driving and aiming || 1 - manual driving, self aiming

PortHandler * portHandler;
PacketHandler * packetHandler;

ros::Time lastMoveCommand;
ros::Time lastShootCommand;

ros::Publisher trigger_pub;
ros::Publisher spin_pub;
ros::Publisher ledControl_pub;

std_msgs::Bool bool_msg;

void toggle_led(int ledID){
  std_msgs::Int8 msg;
  msg.data = ledID;
  ledControl_pub.publish(msg);
}

//Map value from one range into another
float mapValue(float input, float minValInput, float maxValInput, float minValOut, float maxValOut){
  return (input-minValInput)/(maxValInput-minValInput)*(maxValOut-minValOut)+minValOut;
}

bool getPresentPositionCallback(
  dynamixel_sdk_examples::GetPosition::Request & req,
  dynamixel_sdk_examples::GetPosition::Response & res)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  int32_t position = 0;

  dxl_comm_result = packetHandler->read4ByteTxRx(
    portHandler, (uint8_t)req.id, ADDR_PRESENT_POSITION, (uint32_t *)&position, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    //ROS_INFO("getPosition : [ID:%d] -> [POSITION:%d]", req.id, position);
    res.position = position;
    return true;
  } else {
    //ROS_INFO("Failed to get position! Result: %d", dxl_comm_result);
    return false;
  }
}

void setPositionCallback(const dynamixel_sdk_examples::SetPosition::ConstPtr & msg)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  uint32_t position = (unsigned int)msg->position;

  dxl_comm_result = packetHandler->write4ByteTxRx(
    portHandler, (uint8_t)msg->id, ADDR_GOAL_POSITION, position, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    //ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", msg->id, msg->position);
  } else {
    ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
  }
}


//This one tells dynamixels how to move while taking into consideration the position they are currently in
void updateDynamixel(int id, int speed, int minPos, int maxPos){

  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  uint32_t position = 0;                                   //Where am i?
  dxl_comm_result = packetHandler->read4ByteTxRx(
  portHandler, id, ADDR_PRESENT_POSITION, (uint32_t *)&position, &dxl_error);

  if(position >= maxPos){                                 //Am i too far either way?   STOP : move as intended
    speed > 0 ? speed = 0 : speed = speed;
  }
  else if(position <= minPos ){
    speed < 0 ? speed = 0 : speed = speed;
  }

  dxl_comm_result = packetHandler->write4ByteTxRx(
    portHandler, (uint8_t)id, ADDR_GOAL_VELOCITY, speed, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS) {
      //ROS_INFO("setSpeed : [ID:%d] [SPEED:%d] [POS:%d]", id, speed, position);
    } else {
      ROS_ERROR("Failed to set speed! Result: %d", dxl_comm_result);
    }

}

//This one responds to every message sent to the /joy topic
//Here it is used to control the turret
void joyCallback(const sensor_msgs::Joy::ConstPtr & msg){

  if(!msg->buttons[5]){return; }                                //turret enable button

  if(!a_button_pressed && msg->buttons[0]){                     //select mode
    //change used mode
    current_mode++;
    current_mode = current_mode%2;
    toggle_led(GREENLED);
  }
  a_button_pressed = msg->buttons[0];

  //Shooting
  lastShootCommand = ros::Time::now();
  bool_msg.data = (msg->axes[2] < -0.8);       //Left trigger - spin the motors to build inertia if pressed
  spin_pub.publish(bool_msg);

  if( bool_msg.data && (msg->axes[5]<-0.8)){                        //if both Left and RIGHT trigger are pressed - use the other motor to fire the balls
    bool_msg.data = true;
    trigger_pub.publish(bool_msg);
  }
  else{                                                          //else make sure we are not jamming the balls between non spinning motors
    bool_msg.data = false;
    trigger_pub.publish(bool_msg);
  }

  //Pan Tilt

  if(current_mode == 1){return;}                                //If in camera controll mode, don't move

  float msg_speed_x = msg->axes[3];                                 //Pan
  float msg_speed_y = msg->axes[4];                                 //Tilt

  //Pn
  int speed_id2 = mapValue(msg_speed_x,-1,1,40,-40);     
  //Tilt
  int speed_id1 = mapValue(msg_speed_y,-1,1,30,-30);              
  int speed_id3 = mapValue(msg_speed_y,-1,1,-30,30);      

  lastMoveCommand = ros::Time::now();                              //Set the last move command time to current time -> used in the watchdog

  //Pan
  updateDynamixel(2,speed_id2,200,3900);                          //Hardcoded min and max position for pan as the last two parameters :)

  //Tilt
  updateDynamixel(1,speed_id1,1900,2350);                         //Hardcoded min and max position for tilt as the last two parametes :) 
  updateDynamixel(3,speed_id3,1750,2200);
}

void cameraCallback(const geometry_msgs::Twist::ConstPtr & msg){

  if(current_mode == 0){return;}

  float msg_speed_x = msg->angular.z;                               //Pan
  float msg_speed_y = msg->angular.x;                               //Tilt

  int speed_id1 = mapValue(msg_speed_y,-1,1,30,-30);
  int speed_id2 = mapValue(msg_speed_x,-1,1,40,-40);
  int speed_id3 = mapValue(msg_speed_y,-1,1,-30,30);

  lastMoveCommand = ros::Time::now();

  //Pan
  updateDynamixel(2,speed_id2,200,3900);                          //Hardcoded min and max position for pan as the last two parameters :)

  //Tilt
  updateDynamixel(1,speed_id1,1900,2350);                         //Hardcoded min and max position for tilt as the last two parametes :) 
  updateDynamixel(3,speed_id3,1750,2200);
}

//Changes the PID Values while in position control mode
void changePositionPIDValues(int id, uint16_t kp, uint16_t ki, uint16_t kd){

  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler->write2ByteTxRx(
    portHandler, id, ADDR_POS_PID_P, kp, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to change pos kp %d", id);
  }

  dxl_comm_result = packetHandler->write2ByteTxRx(
    portHandler, id, ADDR_POS_PID_I, ki, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to change pos ki %d", id);
  }

  dxl_comm_result = packetHandler->write2ByteTxRx(
    portHandler, id, ADDR_POS_PID_D, kd, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to change pos kd %d", id);
  }

}

//Changes the PI Values while in wheel control mode
void changeVelocityPIValues(int id, uint16_t kp, uint16_t ki){

  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler->write2ByteTxRx(
    portHandler, id, ADDR_VEL_PI_P, kp, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to change vel kp %d", id);
  }

  dxl_comm_result = packetHandler->write2ByteTxRx(
    portHandler, id, ADDR_VEL_PI_I, ki, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to change vel ki %d", id);
  }

}

//Shows the PID Values for position control mode
void readPositionPIDValues(int id){
   uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  uint16_t kp = 0;
  uint16_t ki = 0;
  uint16_t kd = 0;

  dxl_comm_result = packetHandler->read2ByteTxRx(
  portHandler, id, ADDR_POS_PID_P, &kp, &dxl_error);

  dxl_comm_result = packetHandler->read2ByteTxRx(
  portHandler, id, ADDR_POS_PID_I, &ki, &dxl_error);

  dxl_comm_result = packetHandler->read2ByteTxRx(
  portHandler, id, ADDR_POS_PID_D, &kd, &dxl_error);

  ROS_INFO("POS PID: [ID:%d] [P:%d] [I:%d] [D:%d]", id, kp, ki, kd);
}

//Shows the PI Values for wheel control mode
void readVelocityPIValues(int id){
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  uint16_t kp = 0;
  uint16_t ki = 0;

  dxl_comm_result = packetHandler->read2ByteTxRx(
  portHandler, id, ADDR_VEL_PI_P, &kp, &dxl_error);

  dxl_comm_result = packetHandler->read2ByteTxRx(
  portHandler, id, ADDR_VEL_PI_I, &ki, &dxl_error);

  ROS_INFO("VEL PI: [ID:%d] [P:%d] [I:%d]", id, kp, ki);

}

//Makes the dynamixels go to the chosen position 
//This one looks like shit, but works all right.
//It's filled to the brim with hard coded values and repetitive tasks. 
bool homeDynamixelCallback(  dynamixel_sdk_examples::GetPosition::Request & req,  dynamixel_sdk_examples::GetPosition::Response & res){
  //Not yet changed for 3 dynamixels
  return false;

  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  int32_t position = 0;

  if(req.id == 1){position = 1700;}               //hard coded home value for dynamixel ID 1
  else if (req.id == 2){position = 2048;}         //same thing for dynamixel ID 2

  res.position = position;

  //Turn off torque
  dxl_comm_result = packetHandler->write1ByteTxRx(
  portHandler, req.id, ADDR_TORQUE_ENABLE, 0, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to disable torque for Dynamixel ID %d", req.id);
    return -1;
  }

  //Change the mode to position control
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, req.id , ADDR_OPERATING_MODE , 3, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to change to position control ID %d", req.id);
    return false;
  }

  //Turn on torque
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, req.id, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", req.id);
    return -1;
  }

  changePositionPIDValues(req.id, 150, 0, 3600);                  //again hard coded values for a fast but not too snappy homing.
  //ROS_INFO("changed pos PID for: [ID:%d] ", req.id);
  readPositionPIDValues(req.id);

  //Set goal position
  dxl_comm_result = packetHandler->write4ByteTxRx(
    portHandler, req.id, ADDR_GOAL_POSITION, position, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    //ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", req.id, position);
  } else {
    ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
    return false;
  }

  //wait time necessary for the dynamixel to complete the task
  ros::Duration(2.0).sleep();

  //Go back to velocity control

  //Turn off torque
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, req.id, ADDR_TORQUE_ENABLE, 0, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to disable torque for Dynamixel ID %d", req.id);
    return -1;
  }

  //Turn on velocity control
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, req.id, ADDR_OPERATING_MODE , 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to change to position control ID %d", req.id);
    return false;
  }

  //Turn on torque
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, req.id, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", req.id);
    return -1;
  }

  changeVelocityPIValues(req.id, 400, 3000);
  //ROS_INFO("changed vel PI for: [ID:%d]", req.id);
  readVelocityPIValues(req.id);

  return true;
}

int main(int argc, char ** argv)
{

  //ROS setup
  ros::init(argc, argv, "nerf_turret_node");
  ros::NodeHandle nh;
  ros::ServiceServer get_position_srv =     nh.advertiseService("/get_position", getPresentPositionCallback);
  ros::ServiceServer home_dynamixel_srv =   nh.advertiseService("/home_dynamixel", homeDynamixelCallback);
  
  ros::Subscriber set_position_sub =        nh.subscribe("/set_position", 10, setPositionCallback);
  ros::Subscriber set_speed_sub =           nh.subscribe("/joy", 10, joyCallback);
  ros::Subscriber camera_command_sub =      nh.subscribe("/nerf_turret/command",1,cameraCallback);

  trigger_pub =     nh.advertise<std_msgs::Bool>("nerf/toggle_trigger", 1000);
  spin_pub =        nh.advertise<std_msgs::Bool>("nerf/toggle_spin", 1000);
  ledControl_pub =  nh.advertise<std_msgs::Int8>("nerf/control_led",1000);

  //Leds start turned on. Turn them off.
  toggle_led(REDLED);
  toggle_led(BLUELED);
  toggle_led(GREENLED);

  //Dynamixel Setup
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  portHandler = PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  if (!portHandler->openPort()) {
    ROS_ERROR("Failed to open the port!");
    return -1;
  }

  if (!portHandler->setBaudRate(BAUDRATE)) {
    ROS_ERROR("Failed to set the baudrate!");
    return -1;
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL1_ID);
    return -1;
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL2_ID);
    return -1;
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL3_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL3_ID);
    return -1;
  }

  for(int i = 1; i<4;i++){
    changeVelocityPIValues(i, 400, 3000);
    changePositionPIDValues(i, 200, 0, 3600);
    readVelocityPIValues(i);
    readPositionPIDValues(i);
  }

  //Really ugly way of ensuring that the Arduino I've nearly killed is up and running
  //before i send any message to the topics it's subscribing to <-- makes it go craaaaazy
  //60 second delay
  lastMoveCommand = ros::Time::now();
  toggle_led(REDLED);
  while(ros::Time::now() - lastMoveCommand < ros::Duration(60)){};
  toggle_led(REDLED);
  lastMoveCommand = ros::Time::now();
  lastShootCommand = ros::Time::now();
  
  // If the latency between your joy input and the turret moving is large check your USB latency
  // cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer

  // It can be as high as 16. Lower it a bit and everything should work really nicely.
  // Here's how:
  // sudo usermod -aG dialout $USER
  // echo 4 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer

  while(ros::ok())
 {
    //If no command was sent in the last 0.3s
  if((ros::Time::now() - lastMoveCommand) > ros::Duration(0.3)){
    lastMoveCommand = ros::Time::now();

    //stop the dynamixels
    updateDynamixel(2,0,100,4000);
    updateDynamixel(1,0,1900,2350);                          //Hardcoded min and max position for tilt as the last two parametes :) 
    updateDynamixel(3,0,1750,2200);
  }
  if((ros::Time::now() - lastShootCommand) > ros::Duration(0.2)){
    //stop the NERF blaster motors
    bool_msg.data = false;
    trigger_pub.publish(bool_msg);
    spin_pub.publish(bool_msg);
    lastShootCommand = ros::Time::now();
    ROS_WARN("NOW");
   }

   ros::spinOnce();               
 }
  portHandler->closePort();
  return 0;
}
