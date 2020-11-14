// ############### Configuration
#define DEBUG_MODE            1 // no ros communication just some debug prints via serial if set to 1
#define USE_RC_INPUT          0
#define PUBLISH_RC_INPUT      0
#define USE_RC_OUTPUT         0
#define USE_ENCODER_INPUT     0
#define USE_BATTERY_VOLTAGE   1
#define PUBLISH_BATTERY_VOLTAGE 0
#define USE_GYRO              0
#define PUBLISH_GYRO_INPUT    0
#define PUBLISH_VEHICLE_STATE 0
#define PUBLISH_TIMING        0
#define RECEIVE_ROS_COMMAND   0

// ############### Pinout
#define ACCELERATOR_INPUT_PIN 13 // RC receiver channel 2
#define STEERING_INPUT_PIN    14 // RC receiver channel 1
#define ACCELERATOR_OUTPUT_PIN 15// PWM for electronic speed controller
#define STEERING_OUTPUT_PIN   16 // PWM for steering servo
#define BUTTON_PIN            20 // For local input, e.g. shutting down the Pi or stopping/starting recording
#define ENCODER_INPUT_PIN    17 // Pin change interrupt; A0
#define BATTERY_VOLTAGE      21 // For reading 0.5*v_batt; A2
//      SDA                  18 // For gyro MPU6050
//      SCL                  19 // For gyro MPU6050
// Use additional step-down converter to power peripherals; ensure common ground between all devices and do not accidently short-circuit anything. ;-)
#define VOLTAGE_CONVERSION 0.0097969543147208 // Factor from analogRead to real battery voltage (measured through divider)
#define TICKS_PER_REVOLUTION 4

// ############### Includes
#include "ros.h"
#include "ArduinoHardware.h"
#include "limitingcontroller.h"
#include "customdatatypes.h"
#if PUBLISH_VEHICLE_STATE
#include "vehiclestateestimator.h"
#include <rosrccar_messages/VehicleState.h>
#endif
#if USE_RC_INPUT
#include <rosrccar_messages/RCControl.h>
#include "rcreader.h"
#endif
#if USE_RC_OUTPUT
#include <rosrccar_messages/RCControl.h>
#include <Servo.h>
#include "commandarbitrator.h"
#endif
#if USE_ENCODER_INPUT
#include "encoderreader.h"
#include <std_msgs/UInt16.h>
#endif
#if USE_BATTERY_VOLTAGE
#include <std_msgs/Float32.h>
#endif
#if USE_GYRO
#include "mpu6050wrapper.h"
#include <geometry_msgs/Twist.h>
#endif
#include <rosrccar_messages/VehicleCommand.h>
#include "buttonstatemachine.h"

#if DEBUG_MODE
unsigned int loopcounter(0);
#endif
// ############### Global variables & functions
ros::NodeHandle node_handle;
VehicleMeasurement measurements;
rosrccar_messages::VehicleCommand vehiclecommand;
rosrccar_messages::VehicleCommand vehiclecommand_ros;
unsigned long time_last_loop_microseconds(0);
unsigned long time_current_loop_microseconds(0);
unsigned long desired_sample_time_microseconds(20000);
ButtonStateMachine button(BUTTON_PIN);

#if PUBLISH_VEHICLE_STATE
  // rosrccar_messages::VehicleState vehiclestate_msg;
  VehicleStateEstimator estimator;
  #if !DEBUG_MODE
  ros::Publisher vehiclestate_publisher("vehicle_state", &estimator.state);
  #endif
#endif
#if USE_RC_INPUT
  RCReader acceleratorinput(ACCELERATOR_INPUT_PIN);
  RCReader steeringinput(STEERING_INPUT_PIN);
  
  void acceleratorinterrupt() {
    acceleratorinput.processinterrupt();
  }
  void steeringinterrupt() {
    steeringinput.processinterrupt();
  }
  #if PUBLISH_RC_INPUT && !DEBUG_MODE
    rosrccar_messages::RCControl rc_msg;
    ros::Publisher rc_publisher("rc_input", &rc_msg);
  #endif
#endif

#if USE_RC_OUTPUT
  Servo acceleratoroutput;
  Servo steeringoutput;
  CommandArbitrator arbitrator;
#endif
#if RECEIVE_ROS_COMMAND && !DEBUG_MODE
  void rosinterrupt(const rosrccar_messages::VehicleCommand ros_command_received) {
    vehiclecommand_ros = ros_command_received;
  }

  ros::Subscriber<rosrccar_messages::VehicleCommand> roscomand_subscriber("vehicle_command", &rosinterrupt);
#endif

#if USE_ENCODER_INPUT
  EncoderReader encoder(ENCODER_INPUT_PIN, TICKS_PER_REVOLUTION);
  ISR(PCINT1_vect)
  {
    encoder.processinterrupt();
  }
#endif

#if USE_BATTERY_VOLTAGE&&PUBLISH_BATTERY_VOLTAGE && !DEBUG_MODE
  std_msgs::UInt16 battery_msg;
  ros::Publisher battery_publisher("battery_voltage", &battery_msg);
#endif

#if USE_GYRO
  MPU6050Wrapper mpu;
  #if PUBLISH_GYRO_INPUT && !DEBUG_MODE
    geometry_msgs::Twist gyro_msg;
    ros::Publisher gyro_publisher("imu_sensor", &gyro_msg);
  #endif
#endif

#if PUBLISH_TIMING && !DEBUG_MODE
  std_msgs::UInt16 timing_msg;
  ros::Publisher timing_publisher("cycle_time", &timing_msg);
#endif

// ############### Setup
void setup() {
  vehiclecommand.operationmode_lon = manual;
  vehiclecommand.operationmode_lat = manual;
  #if  !DEBUG_MODE
  node_handle.getHardware()->setBaud(256000);
  node_handle.initNode();
  #else
  Serial.begin(9600);
  //delay(1000); // Otherwise the output from Teensy stops after a few lines.
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  #endif
  #if USE_RC_INPUT
    attachInterrupt(digitalPinToInterrupt(ACCELERATOR_INPUT_PIN), acceleratorinterrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(STEERING_INPUT_PIN), steeringinterrupt, CHANGE);
    #if PUBLISH_RC_INPUT && !DEBUG_MODE
      node_handle.advertise(rc_publisher);
    #endif
  #endif

  #if RECEIVE_ROS_COMMAND && !DEBUG_MODE
    node_handle.subscribe(roscomand_subscriber);
  #endif
  
  #if USE_RC_OUTPUT
    acceleratoroutput.attach(ACCELERATOR_OUTPUT_PIN);
    steeringoutput.attach(STEERING_OUTPUT_PIN);
  #endif

  #if USE_ENCODER_INPUT
    cli();
    PCICR |= 0b00000010; // Enables Port C (PCIE1) Pin Change Interrupts
    PCMSK1 |= 0b00000001; // PCINT11 bzw. A3 aktiv, Pins A5-A0
    sei();
  #endif

  #if USE_BATTERY_VOLTAGE&&PUBLISH_BATTERY_VOLTAGE && !DEBUG_MODE
    node_handle.advertise(battery_publisher);
  #endif

  #if USE_GYRO
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    // initialize device
    mpu.initialize();
  
    // load and configure the DMP
    mpu.dmpInitialize(); // hangs if there is no power on MPU6050 (or not connected)
  
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(39);//51
    mpu.setYGyroOffset(-1);//8
    mpu.setZGyroOffset(57);//21
    mpu.setXAccelOffset(-1774);//1150
    mpu.setYAccelOffset(-430);//-50
    mpu.setZAccelOffset(968);//1060
    mpu.setDMPEnabled(true);
    #if PUBLISH_GYRO_INPUT && !DEBUG_MODE
      node_handle.advertise(gyro_publisher);
    #endif
  #endif

  #if PUBLISH_VEHICLE_STATE && !DEBUG_MODE
    node_handle.advertise(vehiclestate_publisher);
  #endif

  #if PUBLISH_TIMING && !DEBUG_MODE
    node_handle.advertise(timing_publisher);
  #endif
}

// ############### Loop
void loop() {
  time_current_loop_microseconds = micros();
  if(time_current_loop_microseconds >= time_last_loop_microseconds+desired_sample_time_microseconds) {
    
    #if USE_RC_INPUT
      measurements.rcaccelerator = acceleratorinput.failsafeinput();
      measurements.rcsteering = steeringinput.failsafeinput();
      #if PUBLISH_RC_INPUT && !DEBUG_MODE
        rc_msg.accelerator = measurements.rcaccelerator;
        rc_msg.rcsteering = measurements.rcsteering;
        rc_msg.valid = acceleratorinput.signalisvalid()&&steeringinput.signalisvalid();
        rc_publisher.publish( &rc_msg );
      #endif
      #if DEBUG_MODE
        loopcounter = loopcounter+1;
        if(loopcounter % 50 == 0) {
          Serial.println("RC Car in Debug Mode");
          Serial.print("RC Input\t ac: ");
          Serial.print(acceleratorinput.failsafeinput());
          Serial.print("\t st: ");
          Serial.print(steeringinput.failsafeinput());
          Serial.println(";");
        }
      #endif
    #endif

    #if USE_ENCODER_INPUT
      measurements.driveshaftspeed_radps = encoder.getangularspeed();
      #if DEBUG_MODE
        if(loopcounter % 50 == 0) {
          Serial.print("Encoder:\t");
          Serial.print(measurements.driveshaftspeed_radps);
          Serial.println(";");
        }
      #endif
    #endif
    
    #if USE_BATTERY_VOLTAGE
      measurements.batteryvoltage_volt = VOLTAGE_CONVERSION*analogRead(BATTERY_VOLTAGE);
      #if PUBLISH_BATTERY_VOLTAGE && !DEBUG_MODE
        battery_msg.data = (unsigned int)(1e3*measurements.batteryvoltage_volt);
        battery_publisher.publish( &battery_msg );
      #endif
      #if DEBUG_MODE
        if(loopcounter % 50 == 0) {
          Serial.print("Battery:\t");
          Serial.print(measurements.batteryvoltage_volt);
          Serial.println(";");
        }
      #endif
    #endif

    #if USE_GYRO
      mpu.update_values();
      measurements.accelerationx_mps2 = mpu.aaReal.x*9.81/8196;
      measurements.accelerationy_mps2 = mpu.aaReal.y*9.81/8196;
      measurements.yaw_rad = mpu.ypr[0];
      #if PUBLISH_GYRO_INPUT && !DEBUG_MODE
        gyro_msg.linear.x = measurements.accelerationx_mps2;
        gyro_msg.linear.y = measurements.accelerationy_mps2;
        gyro_msg.linear.z = mpu.aaReal.z*9.81/8196;
        gyro_msg.angular.z = mpu.ypr[0];
        gyro_msg.angular.x = mpu.ypr[1];
        gyro_msg.angular.y = mpu.ypr[2];
        gyro_publisher.publish( &gyro_msg );
      #endif
      #if DEBUG_MODE
        if(loopcounter % 50 == 0) {
          Serial.print("Gyro: x\t");
          Serial.print(measurements.accelerationx_mps2);
          Serial.print("\ty: ");
          Serial.print(measurements.accelerationy_mps2);
          Serial.print("\tyaw: ");
          Serial.print(measurements.yaw_rad);
          Serial.println(";");
        }
      #endif
    #endif

    #if PUBLISH_VEHICLE_STATE
      estimator.update(measurements, desired_sample_time_microseconds);
      #if !DEBUG_MODE
      vehiclestate_publisher.publish( &estimator.state);
      #endif
    #endif

    #if USE_RC_OUTPUT
      vehiclecommand.target_lon = measurements.rcaccelerator*1e3;
      vehiclecommand.target_lat = measurements.rcsteering*1e3;
      vehiclecommand = arbitrator.arbitrate(vehiclecommand, vehiclecommand_ros);
      steeringoutput.write(float(vehiclecommand.target_lat)/2e3*90+90);
      acceleratoroutput.write(float(vehiclecommand.target_lon)/2e3*90+90);
      estimator.state.steer_command_e3 = vehiclecommand.target_lat;
      estimator.state.acc_command_e3 = vehiclecommand.target_lon;
      estimator.state.operationmode_lon = vehiclecommand.operationmode_lon;
      estimator.state.operationmode_lat = vehiclecommand.operationmode_lat;
    #endif

    #if PUBLISH_VEHICLE_STATE
    if(button.update_status()!=0){
      if(button.current_status==1){
        estimator.state.operationmode_lat = pi_command_start_recording;
      }
      if(button.current_status==3){
        estimator.state.operationmode_lat = pi_command_stop_recording;
      }
      if(button.current_status==3){
        estimator.state.operationmode_lat = pi_command_shutdown;
      }
    }
    #if DEBUG_MODE
      if(loopcounter % 50 == 0) {
        Serial.print("Button:\t");
        Serial.print(button.current_status);
        Serial.println(";");
      }
    #endif
    #endif

    #if PUBLISH_TIMING && !DEBUG_MODE
      timing_msg.data = measurements.looptime_usec;
      timing_publisher.publish( &timing_msg );
    #endif
    #if !DEBUG_MODE
      if(loopcounter % 50 == 0) {
        Serial.print("Timing:\t");
        Serial.print(measurements.looptime_usec);
        Serial.println(";");
      }
    #endif

    #if !DEBUG_MODE
    node_handle.spinOnce();
    measurements.looptime_usec = micros()-time_current_loop_microseconds;
    #endif
    time_last_loop_microseconds = time_current_loop_microseconds;
  }
}
