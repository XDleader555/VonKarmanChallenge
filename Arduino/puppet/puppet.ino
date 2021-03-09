/*
 * puppet - ROS interpreter for angles of puppet
 * Copyright (C) 2018 Andrew Miyaguchi <andrewmiyaguchi@gmail.com>
 * 
 * Recieving communications based off ROS Simple Drive system
 * <https://github.com/danielsnider/simple_drive>
 * 
 */

#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt16MultiArray.h>

#include "Utilities.h"

#define BAUDRATE 250000
#define TOPIC "PuppetArm"
#define NUMINPUTS 6
ros::NodeHandle nh;
std_msgs::UInt16MultiArray angles;
ros::Publisher pub(TOPIC, &angles);

long ros_watchdog;
long nh_watchdog;
long active_watchdog;

uint16_t rawAngles[NUMINPUTS];
uint16_t prevAngles[NUMINPUTS];
uint16_t minAngle[] = {0,
                       0,
                       0,
                       0,
                       180,
                       80};
uint16_t maxAngle[] = {180,
                       180,
                       180,
                       180,
                       0,
                       180};
uint16_t minCal[] = {0,
                     38,
                     0,
                     98,
                     0,
                     235};
uint16_t maxCal[] = {1013,
                     1020,
                     955,
                     880,
                     990,
                     491};


bool readAngles();
bool active = false;

void setup() {
  /* Setup serial connections */
  Serial.begin(BAUDRATE);
  Serial.println(F("Welcome to the Puppet Arm firmware"));
  Serial.println(F("INIT: Firmware booting..."));

  /* Setup the analog inputs */
  dmesg(F("Setting up analog inputs\n"));
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);

  pinMode(7, INPUT_PULLUP);
  
  /* Setup ROS Node */
  dmesg(F("Subscribing to ROS topic\n"));
  Serial.flush();
  
  nh.getHardware()->setBaud(BAUDRATE);
  nh.initNode();

  angles.layout.dim = (std_msgs::MultiArrayDimension *) malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
  //angles.layout.dim_length = 1;
  angles.layout.dim[0].label = "PuppetAngles";
  angles.layout.dim[0].size = NUMINPUTS;
  angles.layout.dim[0].stride = 1*NUMINPUTS;
  angles.layout.data_offset = 0;
  angles.data = (uint16_t *)malloc(sizeof(uint16_t)*NUMINPUTS);
  angles.data_length = NUMINPUTS;
  nh.advertise(pub);
  nh.spinOnce();

  active_watchdog = millis();
  ros_watchdog = millis();
  nh_watchdog = millis();

  //dmesg("Done\n");
}

void loop() {
  if(millis() - active_watchdog > 100) {
    active = digitalRead(7);
    active_watchdog = millis();
  }
  
  /* ROS Loop */
  if(active && millis() - ros_watchdog > 20) {
    if(readAngles()) {
//      Serial.print("Update Detected [");
//      for(int i = 0; i < NUMINPUTS; i ++) {
//        Serial.print(rawAngles[i]);
//        Serial.print(" (");
//        Serial.print(angles.data[i]);
//        Serial.print(")");
//        if(i < NUMINPUTS - 1)
//          Serial.print(", ");
//      }
//      Serial.print("]\n");
      
      pub.publish(&angles);
    }
    ros_watchdog = millis();
  }

  if(millis() - nh_watchdog > 20) {
    nh.spinOnce();
    nh_watchdog = millis();
  }
}

bool readAngles() {
  bool sendPacket = false;
  
  for(int i = 0; i < NUMINPUTS; i ++) {
    prevAngles[i] = angles.data[i];
    rawAngles[i] = analogRead(i);
    angles.data[i] = max(map(rawAngles[i], minCal[i], maxCal[i], minAngle[i], maxAngle[i]), 0);
    
    /* Check if we need to send a new packet */
    if(angles.data[i] != prevAngles[i])
      sendPacket = true;
  }

  return sendPacket;
}
