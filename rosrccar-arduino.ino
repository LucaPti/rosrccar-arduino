// ############### Configuration
#define DEBUG_MODE            0 // no ros communication just some debug prints via serial if set to 1
#define USE_RC_INPUT          1
#define PUBLISH_RC_INPUT      0
#define USE_RC_OUTPUT         1
#define USE_OPTICAL_INPUT     0
#define USE_ENCODER_INPUT     1
#define USE_BATTERY_VOLTAGE   1
#define PUBLISH_BATTERY_VOLTAGE 1
#define USE_GYRO              1
#define PUBLISH_GYRO_INPUT    0
#define PUBLISH_VEHICLE_STATE 1
#define PUBLISH_TIMING        0
#define RECEIVE_ROS_COMMAND   1

// ############### Pinout
#define ACCELERATOR_INPUT_PIN 2 // RC receiver channel 2
#define STEERING_INPUT_PIN    3 // RC receiver channel 1
#define ACCELERATOR_OUTPUT_PIN 5// PWM for electronic speed controller
#define STEERING_OUTPUT_PIN   6 // PWM for steering servo
#define BUTTON_PIN            7 // For local input, e.g. shutting down the Pi or stopping/starting recording
//      SS                   10 // For optical sensor ADNS3050 (ICSP header)
//      MOSI                 11 // For optical sensor ADNS3050 (ICSP header)
//      MISO                 12 // For optical sensor ADNS3050 (ICSP header)
//      SCK                  13 // For optical sensor ADNS3050 (ICSP header)
#define ENCODER_INPUT_PIN    14 // Pin change interrupt; A0
#define BATTERY_VOLTAGE      16 // For reading 0.5*v_batt; A2
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
#if USE_OPTICAL_INPUT
#include "ADNS3050.h"
#endif
#if USE_RC_OUTPUT
#include <rosrccar_messages/RCControl.h>
#include <Servo.h>
#endif
#if USE_ENCODER_INPUT
#include "encoderreader.h"
#include <std_msgs/UInt16.h>
#endif
#if USE_BATTERY_VOLTAGE
#include <std_msgs/Float32.h>
#endif
#if USE_GYRO
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#include "Wire.h"
#include <geometry_msgs/Twist.h>
#endif
#include <rosrccar_messages/VehicleCommand.h>
#include "buttonstatemachine.h"

#if !DEBUG_MODE // Normal "production mode" loop
// ############### Global variables & functions
ros::NodeHandle node_handle;
VehicleMeasurement measurements;
rosrccar_messages::VehicleCommand vehiclecommand;
unsigned long time_last_loop_microseconds(0);
unsigned long time_current_loop_microseconds(0);
unsigned long desired_sample_time_microseconds(20000);
ButtonStateMachine button(BUTTON_PIN);

#if PUBLISH_VEHICLE_STATE
  // rosrccar_messages::VehicleState vehiclestate_msg;
  VehicleStateEstimator estimator;
  ros::Publisher vehiclestate_publisher("vehicle_state", &estimator.state);
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
  #if PUBLISH_RC_INPUT
    rosrccar_messages::RCControl rc_msg;
    ros::Publisher rc_publisher("rc_input", &rc_msg);
  #endif
#endif

#if USE_RC_OUTPUT
  Servo acceleratoroutput;
  Servo steeringoutput;
#endif
#if RECEIVE_ROS_COMMAND
  void rosinterrupt(const rosrccar_messages::VehicleCommand ros_command_received) {
    vehiclecommand = ros_command_received;
  }

  ros::Subscriber<rosrccar_messages::VehicleCommand> roscomand_subscriber("vehicle_command", &rosinterrupt);
#endif

#if USE_OPTICAL_INPUT
  FlexibleIntervalADNS3050 opticalsensor;
#endif

#if USE_ENCODER_INPUT
  EncoderReader encoder(ENCODER_INPUT_PIN, TICKS_PER_REVOLUTION);
  ISR(PCINT1_vect)
  {
    encoder.processinterrupt();
  }
#endif

#if USE_BATTERY_VOLTAGE&&PUBLISH_BATTERY_VOLTAGE
  std_msgs::UInt16 battery_msg;
  ros::Publisher battery_publisher("battery_voltage", &battery_msg);
#endif

#if USE_GYRO
  MPU6050 mpu;
  uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
  uint16_t fifoCount;     // count of all bytes currently in FIFO
  uint8_t fifoBuffer[64]; // FIFO storage buffer
  
  // orientation/motion vars
  Quaternion q;           // [w, x, y, z]         quaternion container
  VectorInt16 aa;         // [x, y, z]            accel sensor measurements
  VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
  VectorFloat gravity;    // [x, y, z]            gravity vector
  float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
  #if PUBLISH_GYRO_INPUT
    geometry_msgs::Twist gyro_msg;
    ros::Publisher gyro_publisher("imu_sensor", &gyro_msg);
  #endif
#endif

#if PUBLISH_TIMING
  std_msgs::UInt16 timing_msg;
  ros::Publisher timing_publisher("cycle_time", &timing_msg);
#endif

// ############### Setup
void setup() {
  vehiclecommand.operationmode_lon = manual;
  vehiclecommand.operationmode_lat = manual;
  node_handle.getHardware()->setBaud(256000);
  node_handle.initNode();
  #if USE_RC_INPUT
    attachInterrupt(digitalPinToInterrupt(ACCELERATOR_INPUT_PIN), acceleratorinterrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(STEERING_INPUT_PIN), steeringinterrupt, CHANGE);
    #if PUBLISH_RC_INPUT
      node_handle.advertise(rc_publisher);
    #endif
  #endif
  
  #if USE_OPTICAL_INPUT
    opticalsensor.startup();
  #endif

  #if RECEIVE_ROS_COMMAND
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

  #if USE_BATTERY_VOLTAGE&&PUBLISH_BATTERY_VOLTAGE
    node_handle.advertise(battery_publisher);
  #endif

  #if USE_GYRO
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    // initialize device
    mpu.initialize();
  
    // load and configure the DMP
    devStatus = mpu.dmpInitialize(); // hangs if there is no power on MPU6050 (or not connected)
  
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(39);//51
    mpu.setYGyroOffset(-1);//8
    mpu.setZGyroOffset(57);//21
    mpu.setXAccelOffset(-1774);//1150
    mpu.setYAccelOffset(-430);//-50
    mpu.setZAccelOffset(968);//1060
    mpu.setDMPEnabled(true);
    #if PUBLISH_GYRO_INPUT
      node_handle.advertise(gyro_publisher);
    #endif
  #endif

  #if PUBLISH_VEHICLE_STATE
    node_handle.advertise(vehiclestate_publisher);
  #endif

  #if PUBLISH_TIMING
    node_handle.advertise(timing_publisher);
  #endif
}

// ############### Loop
void loop() {
  time_current_loop_microseconds = micros();
  if(time_current_loop_microseconds >= time_last_loop_microseconds+desired_sample_time_microseconds) {

    #if USE_OPTICAL_INPUT
      while((!opticalsensor.datavalid())&&(micros()<(time_current_loop_microseconds+3e3))) {}; //wait for optical sensor
      opticalsensor.update();
      while((!opticalsensor.datavalid())&&(micros()<(time_current_loop_microseconds+6e3))) {}; //wait for optical sensor
      opticalsensor.update();
      while((!opticalsensor.datavalid())&&(micros()<(time_current_loop_microseconds+9e3))) {}; //wait for optical sensor
      opticalsensor.update();
      measurements.opticalvelocityx_mps = opticalsensor.velocity_x();
      measurements.opticalvelocityy_mps = opticalsensor.velocity_y();
    #endif
    
    #if USE_RC_INPUT
      measurements.rcaccelerator = acceleratorinput.failsafeinput();
      measurements.rcsteering = steeringinput.failsafeinput();
      #if PUBLISH_RC_INPUT
        rc_msg.accelerator = measurements.rcaccelerator;
        rc_msg.rcsteering = measurements.rcsteering;
        rc_msg.valid = acceleratorinput.signalisvalid()&&steeringinput.signalisvalid();
        rc_publisher.publish( &rc_msg );
      #endif
    #endif

    #if USE_ENCODER_INPUT
      measurements.driveshaftspeed_radps = encoder.getangularspeed();
    #endif
    
    #if USE_BATTERY_VOLTAGE
      measurements.batteryvoltage_volt = VOLTAGE_CONVERSION*analogRead(BATTERY_VOLTAGE);
      #if PUBLISH_BATTERY_VOLTAGE
        battery_msg.data = (unsigned int)(1e3*measurements.batteryvoltage_volt);
        battery_publisher.publish( &battery_msg );
      #endif
    #endif

    #if USE_GYRO
      mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      measurements.accelerationx_mps2 = aaReal.x*9.81/8196;
      measurements.accelerationy_mps2 = aaReal.y*9.81/8196;
      measurements.yaw_rad = ypr[0];
      #if PUBLISH_GYRO_INPUT
        gyro_msg.linear.x = measurements.accelerationx_mps2;
        gyro_msg.linear.y = measurements.accelerationy_mps2;
        gyro_msg.linear.z = aaReal.z*9.81/8196;
        gyro_msg.angular.z = ypr[0];
        gyro_msg.angular.x = ypr[1];
        gyro_msg.angular.y = ypr[2];
        gyro_publisher.publish( &gyro_msg );
      #endif
    #endif

    #if PUBLISH_VEHICLE_STATE
      estimator.update(measurements, desired_sample_time_microseconds);
      vehiclestate_publisher.publish( &estimator.state);
    #endif

    #if USE_RC_OUTPUT
      if((vehiclecommand.operationmode_lon!=manual)&&(measurements.rcaccelerator<-0.5)) {
        vehiclecommand.operationmode_lon = off;
        vehiclecommand.operationmode_lat = off;
      }
      if(vehiclecommand.operationmode_lon==manual){
        acceleratoroutput.write(measurements.rcaccelerator*90*0.5+90); // Calibration: Car reacts only to about 50% of command range
        estimator.state.acc_command_e3 = measurements.rcaccelerator*1e3;
      }
      else if(vehiclecommand.operationmode_lon == automated) {
        acceleratoroutput.write(vehiclecommand.target_lon/1e3*90*0.5+90); // Calibration: Car reacts only to about 50% of command range
        estimator.state.acc_command_e3 = vehiclecommand.target_lon;
      }
      else if(vehiclecommand.operationmode_lon == off) {
        acceleratoroutput.write(90);
        estimator.state.acc_command_e3 = 0;
      }
      else if(vehiclecommand.operationmode_lon == manual_limitedspeed) {
        // to do
      }
      if(vehiclecommand.operationmode_lat==manual){
        steeringoutput.write(measurements.rcsteering*90*0.5+90);
        estimator.state.steer_command_e3 = measurements.rcsteering*1e3;
      }
      else if(vehiclecommand.operationmode_lat == automated) {
        steeringoutput.write(vehiclecommand.target_lat/1e3*90*0.5+90); // Calibration: Car reacts only to about 50% of command range
        estimator.state.steer_command_e3 = vehiclecommand.target_lat;
      }
      else if(vehiclecommand.operationmode_lat == off) {
        steeringoutput.write(90);
        estimator.state.steer_command_e3 = 0;
      }
      estimator.state.operationmode_lon = vehiclecommand.operationmode_lon;
      estimator.state.operationmode_lat = vehiclecommand.operationmode_lat;
    #endif

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

    #if PUBLISH_TIMING
      timing_msg.data = measurements.looptime_usec;
      timing_publisher.publish( &timing_msg );
    #endif
    
    node_handle.spinOnce();
    measurements.looptime_usec = micros()-time_current_loop_microseconds;
    time_last_loop_microseconds = time_current_loop_microseconds;
  }
}

#else // DEBUG MODE, no ROS communication, just prints via serial #####################################################################################################################################
// ############### Global variables & functions
#if USE_RC_INPUT
RCReader acceleratorinput(ACCELERATOR_INPUT_PIN);
RCReader steeringinput(STEERING_INPUT_PIN);

void acceleratorinterrupt() {
  acceleratorinput.processinterrupt();
}
void steeringinterrupt() {
  steeringinput.processinterrupt();
}
#endif
#if USE_RC_OUTPUT
Servo acceleratoroutput;
Servo steeringoutput;
#endif

#if USE_ENCODER_INPUT
EncoderReader encoder(ENCODER_INPUT_PIN, TICKS_PER_REVOLUTION);
ISR(PCINT1_vect)
{
  encoder.processinterrupt();
}
#endif

#if USE_OPTICAL_INPUT
FlexibleIntervalADNS3050 opticalsensor;
#endif

#if USE_GYRO
MPU6050 mpu;
// MPU control/status vars
//bool dmpReady = false;  // set true if DMP init was successful
//uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
//uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
//VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
//VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
//float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
//void dmpDataReady() {
//  mpuInterrupt = true;
//}
#endif


// ############### Setup
void setup() {
  Serial.begin(115200);
  #if USE_RC_INPUT
  attachInterrupt(digitalPinToInterrupt(ACCELERATOR_INPUT_PIN), acceleratorinterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(STEERING_INPUT_PIN), steeringinterrupt, CHANGE);
  #endif
  
  #if USE_OPTICAL_INPUT
  opticalsensor.startup();
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

  #if USE_GYRO
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  //#define INTERRUPT_PIN 3
  //pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  //while (Serial.available() && Serial.read()); // empty buffer
  //while (!Serial.available());                 // wait for data
  //while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize(); // hangs if there is no power on MPU6050 (or not connected)

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(39);//51
  mpu.setYGyroOffset(-1);//8
  mpu.setZGyroOffset(57);//21
  mpu.setXAccelOffset(-1774);//1150
  mpu.setYAccelOffset(-430);//-50
  mpu.setZAccelOffset(968);//1060
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println();
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    //Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    //Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    //Serial.println(F(")..."));
    //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    //mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    // Serial.println(F("DMP ready! Waiting for first interrupt..."));
    // dmpReady = true;

    // get expected DMP packet size for later comparison
    //packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  #endif
}
// ############### Loop
void loop() {
  Serial.println();
  Serial.println("RC car in debug mode.");
// RC Input
#if USE_RC_INPUT
  Serial.print("RC Input\t ac: ");
  Serial.print(acceleratorinput.failsafeinput());
  Serial.print(" ");
  (acceleratorinput.signalisvalid()) ? Serial.print("OK") : Serial.print("ERR");
  Serial.print("\t st: ");
  Serial.print(steeringinput.failsafeinput());
  Serial.print(" ");
  (steeringinput.signalisvalid()) ? Serial.print("OK") : Serial.print("ERR");
  Serial.println(";");
#endif
// RC Output
#if (USE_RC_OUTPUT&&USE_RC_INPUT)
  Serial.println("Applying steering to servo.");
  steeringoutput.write(steeringinput.failsafeinput()*90*0.5+90);
#endif
// Optical Sensor
#if USE_OPTICAL_INPUT
  opticalsensor.update();
  delay(3);
  opticalsensor.update();
  delay(3);
  opticalsensor.update();
  Serial.print("Optical sensor\t x: ");
  Serial.print(opticalsensor.velocity_x());
  Serial.print(" y: ");
  Serial.print(opticalsensor.velocity_y());
  (opticalsensor.lastvaluevalid()) ? Serial.print(" OK") : Serial.print(" ERR");
  Serial.print(" quality: ");
  Serial.println(opticalsensor.measurementquality());
#endif
// Encoder
#if USE_ENCODER_INPUT
  Serial.print("Encoder: ");
  Serial.println(encoder.getangularspeed());
#endif
// Battery
#if USE_BATTERY_VOLTAGE
  Serial.print("Voltage: ");
  Serial.println(VOLTAGE_CONVERSION*analogRead(BATTERY_VOLTAGE));
#endif
// Gyro
#if USE_GYRO
  mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
  Serial.print("Gyro: ");
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  Serial.print("ypr\t");
  Serial.print(ypr[0] * 180 / M_PI);
  Serial.print("\t");
  Serial.print(ypr[1] * 180 / M_PI);
  Serial.print("\t");
  Serial.println(ypr[2] * 180 / M_PI);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  Serial.print("Accel: xyz\t");
  Serial.print(aaReal.x*9.81/8196);
  Serial.print("\t");
  Serial.print(aaReal.y*9.81/8196);
  Serial.print("\t");
  Serial.println(aaReal.z*9.81/8196);
#endif
delay(250);
}
#endif
