
// Sorry for making this thing one long file. Future me will fix it.
// At least it's not like I've released the files so you can recreate the turret


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
#define DXL1_ID               1               // DXL1 ID - tilt left
#define DXL2_ID               2               // DXL2 ID - pan
#define DXL3_ID               3               // DXL3 ID - tilt right

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
  /**
   * Sends a message ment to toggle chosen LED
  */

  std_msgs::Int8 msg;
  msg.data = ledID;
  ledControl_pub.publish(msg);
}

//Map value from one range into another
float mapValue(float input, float minValInput, float maxValInput, float minValOut, float maxValOut){
  /**
   * Maps input value from input range to output range
   * @param input - value to map
   * @param minValInput - input range min Value
   * @param maxValInput - input range max Value
   * @param minValOut - input range min Value
   * @param maxValOut - input range min Value
   * @return mapped value
  */
  return (input-minValInput)/(maxValInput-minValInput)*(maxValOut-minValOut)+minValOut;
}

void updateDynamixel(int id, int speed, int minPos, int maxPos){
  /**
   *  Used to tell dynamixels how to move. Uses min/max pos to stop force the dynamixels to stop when moving out of bounds
   * @param id - dynamixel ID
   * @param speed - speed to move at (positive or negative depending on the direction)
   * @param minPos - lower positional bound
   * @param maxPos - upper positional bound
  */

  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // Check current position
  uint32_t position = 0;
  dxl_comm_result = packetHandler->read4ByteTxRx(
  portHandler, id, ADDR_PRESENT_POSITION, (uint32_t *)&position, &dxl_error);
  // Compare with bounds, speed = 0 if OOB. Continue otherwise
  if(position >= maxPos)          {speed > 0 ? speed = 0 : speed = speed;}
  else if(position <= minPos )    {speed < 0 ? speed = 0 : speed = speed;}

  // Update dynamixel speed
  dxl_comm_result = packetHandler->write4ByteTxRx(
    portHandler, (uint8_t)id, ADDR_GOAL_VELOCITY, speed, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS) {
      //ROS_INFO("setSpeed : [ID:%d] [SPEED:%d] [POS:%d]", id, speed, position);
    } else {
      ROS_ERROR("Failed to set speed! Result: %d", dxl_comm_result);
    }

}

void joyCallback(const sensor_msgs::Joy::ConstPtr & msg){

  /**
   * Reacts to joy message.
   * Moves / changes state of the turret if requested
  */

  // turret control enable button
  if(!msg->buttons[5]){return; }                                

  // turret operating mode selection
  if(!a_button_pressed && msg->buttons[0]){                     
    current_mode++;
    current_mode = current_mode%2;
    toggle_led(GREENLED);
  }
  a_button_pressed = msg->buttons[0];

  //Shooting
  lastShootCommand = ros::Time::now();
  bool_msg.data = (msg->axes[2] < -0.8);                      //Left trigger - spin the motors to build inertia if pressed
  spin_pub.publish(bool_msg);

  if( bool_msg.data && (msg->axes[5]<-0.8)){                  //if both Left and right triggers are pressed - use the other motor to fire the balls
    bool_msg.data = true;
    trigger_pub.publish(bool_msg);
  }
  else{                                                       //else make sure we are not jamming the balls between non spinning motors
    bool_msg.data = false; 
    trigger_pub.publish(bool_msg);
  }

  //Pan Tilt
  if(current_mode == 1){return;}                               //If in autonomous control mode, return

  lastMoveCommand = ros::Time::now();                             //Set the last move command time to current time -> used in the watchdog

  float msg_speed_x = msg->axes[3];
  float msg_speed_y = msg->axes[4];

  //Pn
  int speed_id2 = mapValue(msg_speed_x,-1,1,40,-40);              //Hardcoded motor speed as the last two parameters :) 
  //Tilt
  int speed_id1 = mapValue(msg_speed_y,-1,1,30,-30);              
  int speed_id3 = mapValue(msg_speed_y,-1,1,-30,30);      

  //Pan
  updateDynamixel(2,speed_id2,200,3900);                          //Hardcoded min and max position for pan as the last two parameters :)
  //Tilt
  updateDynamixel(1,speed_id1,1900,2350);
  updateDynamixel(3,speed_id3,1750,2200);
}

void cameraCallback(const geometry_msgs::Twist::ConstPtr & msg){
  /**
   * Reacts to target detector message.
   * Makes the dynamixels move at the speed requested by it.
  */

  if(current_mode == 0){return;}                                //If not in autonoums control mode, return

  float msg_speed_x = msg->angular.z;
  float msg_speed_y = msg->angular.x;

  int speed_id1 = mapValue(msg_speed_y,-1,1,30,-30);              //Hardcoded motor speed as the last two parameters :) 
  int speed_id2 = mapValue(msg_speed_x,-1,1,40,-40);
  int speed_id3 = mapValue(msg_speed_y,-1,1,-30,30);

  lastMoveCommand = ros::Time::now();

  //Pan
  updateDynamixel(2,speed_id2,200,3900);                          //Hardcoded min and max position for pan as the last two parameters :)
  //Tilt
  updateDynamixel(1,speed_id1,1900,2350);
  updateDynamixel(3,speed_id3,1750,2200);
}

void changePositionPIDValues(int id, uint16_t kp, uint16_t ki, uint16_t kd){
  /**
   * Changes the dynamixel PID Values in position control mode
  */

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

void changeVelocityPIValues(int id, uint16_t kp, uint16_t ki){
  /**
   * Changes the dynamixel PI Values in speed control mode
  */

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

void readPositionPIDValues(int id){
  /**
   * Reads and shows the dynamixel position control PID Values
  */

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

void readVelocityPIValues(int id){
  /**
   * Reads and shows the dynamixel speed control PI Values
  */
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

int main(int argc, char ** argv)
{

  //ROS setup
  ros::init(argc, argv, "nerf_turret_node");
  ros::NodeHandle nh;
  
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

  //This should not be repeated 3 times
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

  // Really ugly way of ensuring that the Arduino I've nearly killed is up and running
  // before i send any message to the topics it's subscribing to <-- makes it go craaaaazy
  // 60 second delay
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
  //If no joy command was sent in the last 0.3s
  if((ros::Time::now() - lastMoveCommand) > ros::Duration(0.3)){
    lastMoveCommand = ros::Time::now();

    //stop the pan//tilt dynamixels
    updateDynamixel(DXL2_ID,0,100,4000);
    updateDynamixel(DXL1_ID,0,1900,2350);                          //Hardcoded min and max position for tilt as the last two parametes :) 
    updateDynamixel(DXL3_ID,0,1750,2200);
    }
  if((ros::Time::now() - lastShootCommand) > ros::Duration(0.2)){
    //stop the NERF blaster motors
    bool_msg.data = false;
    trigger_pub.publish(bool_msg);
    spin_pub.publish(bool_msg);
    lastShootCommand = ros::Time::now();
  }

  ros::spinOnce();               
 }
  portHandler->closePort();
  return 0;
}
