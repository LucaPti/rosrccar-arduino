#include <ros.h>
//#include <ArduinoTcpHardware.h>
#include <ArduinoHardware.h>
#include <arduino_messages/RCControl.h>
#include <arduino_messages/RawOpticalSensorData.h>

#include <Servo.h>
#include "rcreader.h"

#include "ADNS3050.h"

#define ACCELERATOR_INPUT_PIN 2
#define STEERING_INPUT_PIN    3
#define ACCELERATOR_OUTPUT_PIN 5
#define STEERING_OUTPUT_PIN   6
RCReader acceleratorinput(ACCELERATOR_INPUT_PIN);
RCReader steeringinput(STEERING_INPUT_PIN);
Servo acceleratoroutput;
Servo steeringoutput;

void acceleratorinterrupt() {
  acceleratorinput.processinterrupt();
}
void steeringinterrupt() {
  steeringinput.processinterrupt();
}
void rosinterrupt(const arduino_messages::RCControl ros_comand) {
  if(ros_comand.valid) {
    acceleratoroutput.write(ros_comand.accelerator*90-90);
    steeringoutput.write(ros_comand.accelerator*90-90);
  }
}

ros::NodeHandle node_handle;

arduino_messages::RCControl rc_msg;
arduino_messages::RawOpticalSensorData optsens_msg;

ros::Publisher rc_publisher("rc_input", &rc_msg);
ros::Publisher optsens_publisher("optical_sensor", &optsens_msg);
ros::Subscriber<arduino_messages::RCControl> roscomand_subscriber("rc_output", &rosinterrupt);

void setup() {
  node_handle.initNode();
  node_handle.advertise(rc_publisher);
  node_handle.advertise(optsens_publisher);
  node_handle.subscribe(roscomand_subscriber);
  
  attachInterrupt(digitalPinToInterrupt(ACCELERATOR_INPUT_PIN), acceleratorinterrupt, CHANGE);
  acceleratoroutput.attach(ACCELERATOR_OUTPUT_PIN);
  
  attachInterrupt(digitalPinToInterrupt(STEERING_INPUT_PIN), steeringinterrupt, CHANGE);
  steeringoutput.attach(STEERING_OUTPUT_PIN);

  adns3050::startup();
}

void loop() {
  rc_msg.accelerator = acceleratorinput.failsafeinput();
  rc_msg.steering = steeringinput.failsafeinput();
  rc_msg.valid = acceleratorinput.signalisvalid()&&steeringinput.signalisvalid();

  rc_publisher.publish( &rc_msg );

  optsens_msg.delta_x = adns3050::getX();
  optsens_msg.delta_y = adns3050::getY();

  optsens_publisher.publish( &optsens_msg );
  node_handle.spinOnce();

  delay(100);
}
