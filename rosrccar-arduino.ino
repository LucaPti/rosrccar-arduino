#define USE_RC_INPUT 0
#define USE_RC_OUTPUT 0
#define USE_OPTICAL_INPUT 1

#include <ros.h>
//#include <ArduinoTcpHardware.h>
#include <ArduinoHardware.h>
#if USE_RC_INPUT
#include <arduino_messages/RCControl.h>
#include "rcreader.h"
#define ACCELERATOR_INPUT_PIN 2
#define STEERING_INPUT_PIN    3
#endif
#if USE_OPTICAL_INPUT
#include <arduino_messages/RawOpticalSensorData.h>
#include "ADNS3050.h"
#endif
#if USE_RC_OUTPUT
#include <Servo.h>
#endif

ros::NodeHandle node_handle;
unsigned long time_last_loop_microseconds(0);
unsigned long time_current_loop_microseconds(0);
unsigned long desired_sample_time_microseconds(10000);

#if USE_RC_INPUT
RCReader acceleratorinput(ACCELERATOR_INPUT_PIN);
RCReader steeringinput(STEERING_INPUT_PIN);

void acceleratorinterrupt() {
  acceleratorinput.processinterrupt();
}
void steeringinterrupt() {
  steeringinput.processinterrupt();
}

arduino_messages::RCControl rc_msg;

ros::Publisher rc_publisher("rc_input", &rc_msg);
#endif

#if USE_RC_OUTPUT
#define ACCELERATOR_OUTPUT_PIN 5
#define STEERING_OUTPUT_PIN   6
Servo acceleratoroutput;
Servo steeringoutput;

void rosinterrupt(const arduino_messages::RCControl ros_comand) {
  if(ros_comand.valid) {
    acceleratoroutput.write(ros_comand.accelerator*90-90);
    steeringoutput.write(ros_comand.accelerator*90-90);
  }
}

ros::Subscriber<arduino_messages::RCControl> roscomand_subscriber("rc_output", &rosinterrupt);
#endif

#if USE_OPTICAL_INPUT
arduino_messages::RawOpticalSensorData optsens_msg;
ros::Publisher optsens_publisher("optical_sensor", &optsens_msg);
#endif

void setup() {
  node_handle.initNode();
  #if USE_RC_INPUT
  attachInterrupt(digitalPinToInterrupt(ACCELERATOR_INPUT_PIN), acceleratorinterrupt, CHANGE);
  acceleratoroutput.attach(ACCELERATOR_OUTPUT_PIN);
  
  attachInterrupt(digitalPinToInterrupt(STEERING_INPUT_PIN), steeringinterrupt, CHANGE);
  steeringoutput.attach(STEERING_OUTPUT_PIN);
  
  node_handle.advertise(rc_publisher);
  #endif
  
  #if USE_OPTICAL_INPUT
  adns3050::startup();
  node_handle.advertise(optsens_publisher);
  #endif
  
  #if USE_RC_OUTPUT
  node_handle.subscribe(roscomand_subscriber);
  #endif
}

void loop() {
  time_current_loop_microseconds = micros();
  if(time_current_loop_microseconds >= time_last_loop_microseconds+desired_sample_time_microseconds) {
    #if USE_RC_INPUT
    rc_msg.accelerator = acceleratorinput.failsafeinput();
    rc_msg.steering = steeringinput.failsafeinput();
    rc_msg.valid = acceleratorinput.signalisvalid()&&steeringinput.signalisvalid();
  
    rc_publisher.publish( &rc_msg );
    #endif
  
    #if USE_OPTICAL_INPUT
    optsens_msg.delta_x = adns3050::getX();
    optsens_msg.delta_y = adns3050::getY();
  
    optsens_publisher.publish( &optsens_msg );
    #endif
    
    node_handle.spinOnce();
  }
}
