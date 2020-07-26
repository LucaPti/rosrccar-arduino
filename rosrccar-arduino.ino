// ############### Configuration
#define DEBUG_MODE            1 // no ros communication just some debug prints via serial if set to 1
#define USE_RC_INPUT          1
#define USE_RC_OUTPUT         1
#define USE_OPTICAL_INPUT     1
#define USE_ENCODER_INPUT     1
#define RELAY_RC_COMMAND      1
#define USE_BATTERY_VOLTAGE   1
#define USE_GYRO              1

// ############### Pinout
#define ACCELERATOR_INPUT_PIN 2 // RC receiver channel 2
#define STEERING_INPUT_PIN    3 // RC receiver channel 1
#define ACCELERATOR_OUTPUT_PIN 5// PWM for electronic speed controller
#define STEERING_OUTPUT_PIN   6 // PWM for steering servo
//      SS                   10 // For optical sensor ADNS3050 (ICSP header)
//      MOSI                 11 // For optical sensor ADNS3050 (ICSP header)
//      MISO                 12 // For optical sensor ADNS3050 (ICSP header)
//      SCK                  13 // For optical sensor ADNS3050 (ICSP header)
//      ENCODER_INPUT_PIN    14 // Pin change interrupt; A0
#define BATTERY_VOLTAGE      16 // For reading 0.5*v_batt; A2
//      SDA                  18 // For gyro MPU6050
//      SCL                  19 // For gyro MPU6050
// Use additional step-down converter to power peripherals; ensure common ground between all devices and do not accidently short-circuit anything. ;-)
#define VOLTAGE_CONVERSION 0.0097969543147208 // Factor from analogRead to real battery voltage (measured through divider)

// ############### Includes
#include "ros.h"
#include "ArduinoHardware.h"
#if USE_RC_INPUT
#include <rosrccar_messages/RCControl.h>
#include "rcreader.h"
#endif
#if USE_OPTICAL_INPUT
#include <rosrccar_messages/RawOpticalSensorData.h>
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
#if USE_GYRO
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#include "Wire.h"
#endif

#if !DEBUG_MODE // Normal "production mode" loop
// ############### Global variables & functions
ros::NodeHandle node_handle;
unsigned long time_last_loop_microseconds(0);
unsigned long time_current_loop_microseconds(0);
unsigned long desired_sample_time_microseconds(20000);

#if USE_RC_INPUT
RCReader acceleratorinput(ACCELERATOR_INPUT_PIN);
RCReader steeringinput(STEERING_INPUT_PIN);

void acceleratorinterrupt() {
  acceleratorinput.processinterrupt();
}
void steeringinterrupt() {
  steeringinput.processinterrupt();
}

rosrccar_messages::RCControl rc_msg;

ros::Publisher rc_publisher("rc_input", &rc_msg);
#endif

#if USE_RC_OUTPUT
Servo acceleratoroutput;
Servo steeringoutput;

void rosinterrupt(const rosrccar_messages::RCControl ros_comand) {
  if(ros_comand.valid) {
#if !(RELAY_RC_COMMAND)
    acceleratoroutput.write(ros_comand.accelerator*90*0.5+90); // Calibration: Car reacts only to about 50% of command range
    steeringoutput.write(ros_comand.steering*90*0.5+90);
#endif
  }
}

ros::Subscriber<rosrccar_messages::RCControl> roscomand_subscriber("rc_output", &rosinterrupt);
#endif

#if USE_OPTICAL_INPUT
rosrccar_messages::RawOpticalSensorData optsens_msg;
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

// ############### Setup
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

// ############### Loop
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
    optsens_msg.valid = adns3050::datavalid();
    if(optsens_msg.valid){
      optsens_msg.delta_x = adns3050::getX();
      optsens_msg.delta_y = adns3050::getY();
    }
    else {
      optsens_msg.delta_x = 0;
      optsens_msg.delta_y = adns3050::measurementquality();
    }
    optsens_publisher.publish( &optsens_msg );
    #endif

    #if USE_ENCODER_INPUT
    encoder_msg.data = encoder.totalticks();

    encoder_publisher.publish( &encoder_msg );
    #endif
    #if RELAY_RC_COMMAND
    acceleratoroutput.write(rc_msg.accelerator*90*0.5+90); // Calibration: Car reacts only to about 50% of command range
    steeringoutput.write(rc_msg.steering*90*0.5+90);
    #else
    // RC output handled via interrupt
    #endif
    
    node_handle.spinOnce();
    time_last_loop_microseconds = time_current_loop_microseconds;
  }
}

#else // DEBUG MODE, no ROS communication, just prints via serial
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
EncoderReader encoder;
ISR(PCINT1_vect)
{
  if(~digitalRead(A0)) // only trigger on falling edge
  {
    encoder++;
  }
}
#endif

#if USE_GYRO
MPU6050 mpu;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}
#endif


// ############### Setup
void setup() {
  Serial.begin(115200);
  #if USE_RC_INPUT
  attachInterrupt(digitalPinToInterrupt(ACCELERATOR_INPUT_PIN), acceleratorinterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(STEERING_INPUT_PIN), steeringinterrupt, CHANGE);
  #endif
  
  #if USE_OPTICAL_INPUT
  adns3050::startup();
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
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    // Serial.println(F("DMP ready! Waiting for first interrupt..."));
    // dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
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
#if (USE_RC_OUTPUT&&RELAY_RC_COMMAND&&USE_RC_INPUT)
  Serial.println("Applying steering to servo.");
  steeringoutput.write(steeringinput.failsafeinput()*90*0.5+90);
#endif
// Optical Sensor
#if USE_OPTICAL_INPUT
  Serial.print("Optical sensor\t x: ");
  Serial.print(adns3050::getX());
  Serial.print(" y: ");
  Serial.print(adns3050::getY());
  (adns3050::datavalid()) ? Serial.print(" OK") : Serial.print(" ERR");
  Serial.print(" quality: ");
  Serial.println(adns3050::measurementquality());
#endif
// Encoder
#if USE_ENCODER_INPUT
  Serial.print("Encoder: ");
  Serial.println(encoder.totalticks());
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
  Serial.print(aaReal.x);
  Serial.print("\t");
  Serial.print(aaReal.y);
  Serial.print("\t");
  Serial.println(aaReal.z);
#endif
delay(1000);
}
#endif
