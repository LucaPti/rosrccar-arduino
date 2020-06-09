#define USE_RC_INPUT 1
#define USE_RC_OUTPUT 1
#define USE_OPTICAL_INPUT 0
#define USE_ENCODER_INPUT 1

#include "ros.h"
#include "ArduinoHardware.h"
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
#if USE_ENCODER_INPUT
#include "encoderreader.h"
#define ENCODER_INPUT_PIN 14 //A0, uses pin change interrupt so here just for documentation
#include <std_msgs/UInt16.h>
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
    acceleratoroutput.write(ros_comand.accelerator*90+90);
    steeringoutput.write(ros_comand.steering*90+90);
  }
}

ros::Subscriber<arduino_messages::RCControl> roscomand_subscriber("rc_output", &rosinterrupt);
#endif

#if USE_OPTICAL_INPUT
arduino_messages::RawOpticalSensorData optsens_msg;
ros::Publisher optsens_publisher("optical_sensor", &optsens_msg);
#endif

#if USE_ENCODER_INPUT
EncoderReader encoder;
ISR(PCINT1_vect)
{
  if(~digitalRead(A0)) // only trigger on falling edge
  {
    encoder++;
  }
}
std_msgs::UInt16 encoder_msg;
ros::Publisher encoder_publisher("encoder_sensor", &encoder_msg);
#endif

void setup() {
  node_handle.getHardware()->setBaud(256000);
  node_handle.initNode();
  #if USE_RC_INPUT
  attachInterrupt(digitalPinToInterrupt(ACCELERATOR_INPUT_PIN), acceleratorinterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(STEERING_INPUT_PIN), steeringinterrupt, CHANGE);
  
  node_handle.advertise(rc_publisher);
  #endif
  
  #if USE_OPTICAL_INPUT
  adns3050::startup();
  node_handle.advertise(optsens_publisher);
  #endif
  
  #if USE_RC_OUTPUT
  acceleratoroutput.attach(ACCELERATOR_OUTPUT_PIN);
  steeringoutput.attach(STEERING_OUTPUT_PIN);
  node_handle.subscribe(roscomand_subscriber);
  #endif

  #if USE_ENCODER_INPUT
  cli();
  PCICR |= 0b00000010; // Enables Port C (PCIE1) Pin Change Interrupts
  PCMSK1 |= 0b00000001; // PCINT11 bzw. A3 aktiv, Pins A5-A0
  sei();
  node_handle.advertise(encoder_publisher);
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

    #if USE_ENCODER_INPUT
    encoder_msg.data = encoder.totalticks();

    encoder_publisher.publish( &encoder_msg );
    #endif
    // RC output handled via interrupt
    
    node_handle.spinOnce();
    time_last_loop_microseconds = time_current_loop_microseconds;
  }
}
