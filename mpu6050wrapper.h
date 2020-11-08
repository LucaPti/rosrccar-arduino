#ifndef MPU6050WRAPPER
#define MPU6050WRAPPER

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#include "Wire.h"

class MPU6050Wrapper : public MPU6050 {
  public:
    uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer
  
    // orientation/motion vars
    Quaternion q;           // [w, x, y, z]         quaternion container
    VectorInt16 aa;         // [x, y, z]            accel sensor measurements
    VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
    VectorFloat gravity;    // [x, y, z]            gravity vector
    float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
    void update_values();
};

void MPU6050Wrapper::update_values() {
    dmpGetCurrentFIFOPacket(fifoBuffer);
    dmpGetQuaternion(&q, fifoBuffer);
    dmpGetGravity(&gravity, &q);
    dmpGetYawPitchRoll(ypr, &q, &gravity);
    dmpGetAccel(&aa, fifoBuffer);
    dmpGetLinearAccel(&aaReal, &aa, &gravity);
}
#endif
