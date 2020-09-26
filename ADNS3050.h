#ifndef ADNS3050
#define ADNS3050

#include <SPI.h>
 #define NUMBEROFVALUESTOSTORE 4
 #define COUNTSPERMETER_DEFAULT 250/0.0254

 // SPI and misc pins for the ADNS
 #define PIN_SCLK   SCK
 #define PIN_MISO   MISO
 #define PIN_MOSI   MOSI
 #define PIN_NCS    10

// Define all the registers
 #define PROD_ID                          0x00
 #define REV_ID                           0x01
 #define MOTION_ST                        0x02
 #define DELTA_X                          0x03
 #define DELTA_Y                          0x04
 #define SQUAL                            0x05
 #define SHUT_HI                          0x06
 #define SHUT_LO                          0x07
 #define PIX_MAX                          0x08
 #define PIX_ACCUM                        0x09
 #define PIX_MIN                          0x0a
 #define PIX_GRAB                         0x0b
 #define MOUSE_CTRL                       0x0d
 #define RUN_DOWNSHIFT                    0x0e
 #define REST1_PERIOD                     0x0f
 #define REST1_DOWNSHIFT                  0x10
 #define REST2_PERIOD                     0x11
 #define REST2_DOWNSHIFT                  0x12
 #define REST3_PERIOD                     0x13
 #define PREFLASH_RUN                     0x14
 #define PREFLASH_RUN_DARK                0x18
 #define MOTION_EXT                       0x1b
 #define SHUT_THR                         0x1c
 #define SQUAL_THRESHOLD                  0x1d
 #define NAV_CTRL2                        0x22
 #define MISC_SETTINGS                    0x25
 #define RESOLUTION                       0x33
 #define LED_PRECHARGE                    0x34
 #define FRAME_IDLE                       0x35
 #define RESET                            0x3a
 #define INV_REV_ID                       0x3f
 #define LED_CTRL                         0x40
 #define MOTION_CTRL                      0x41
 #define AUTO_LED_CONTROL                 0x43
 #define REST_MODE_CONFIG                 0x45

class ADNS3050Reader {
  protected:
    void com_start(){
      digitalWrite(PIN_NCS, HIGH);
      delay(20);
      digitalWrite(PIN_NCS, LOW);
    }
    byte Read(byte reg_addr){
      digitalWrite(PIN_NCS, LOW);//begin communication
      // send address of the register, with MSBit = 0 to say it's reading
      SPI.transfer(reg_addr & 0x7f );
      delayMicroseconds(5);
      // read data
      byte data = SPI.transfer(0);
      delayMicroseconds(1);
      digitalWrite(PIN_NCS, HIGH);//end communication
      delayMicroseconds(1);

      return data;
    }
    void Write(byte reg_addr, byte data){
      digitalWrite(PIN_NCS, LOW);
      //send address of the register, with MSBit = 1 to say it's writing
      SPI.transfer(reg_addr | 0x80 );
      //send data
      SPI.transfer(data);
      delayMicroseconds(30);
      digitalWrite(PIN_NCS, HIGH);//end communication
      delayMicroseconds(30);
    }
    int convTwosComp(int b){ //Convert from 2's complement
      if(b & 0x80){
        b = -1 * ((b ^ 0xff) + 1);
        }
      return b;
    }
    int last_x;
    int last_y;
  public:
    void startup(){
      //--------Setup SPI Communication---------
      //Serial.begin(115200);
      byte out = 0;
      byte read = 0;
      byte bit = 0;
      pinMode(PIN_MISO,INPUT);
      pinMode(PIN_NCS,OUTPUT);
      SPI.begin();
      // set the details of the communication
      SPI.setBitOrder(MSBFIRST); // transimission order of bits
      SPI.setDataMode(SPI_MODE3); // sampling on rising edge
      SPI.setClockDivider(SPI_CLOCK_DIV16); // 16MHz/16 = 1MHz
      delay(10);

      //----------------- Power Up and config ---------------
      com_start();
      Write(RESET, 0x5a); // force reset
      delay(100); // wait for it to reboot
      Write(MOUSE_CTRL, 0x24);//Setup Mouse Control; 20 sets resolution to 1000dpi, 24 choses 250dpi
      Write(MOTION_CTRL, 0x00);//Clear Motion Control register
      delay(100);
    }
    void update(){
      last_x = getX();
      last_y = getY();
    }
    int x() const {return last_x; }
    int y() const {return last_y; }
    unsigned int measurementquality(){
      byte m = 0;
      m = Read(SQUAL);
      return (static_cast<unsigned int>(m)<<1);
    }
  protected:
    int getX(){//returns the X acceleration value
      byte x = 0;
      x= Read(0x03);
      return(convTwosComp(x));
    }
    int getY(){//returns the Y acceleration value
      byte y = 0;
      y= Read(0x04);
      return(convTwosComp(y));
    }
    bool datavalid(){
      byte s = 0;
      s = Read(MOTION_ST);
      return((s&(1<<7))!=0);
    }
};

class FlexibleIntervalADNS3050 : public ADNS3050Reader {
  private:
    unsigned long lastupdatetime;
    long updatetimes[NUMBEROFVALUESTOSTORE];
    int xval[NUMBEROFVALUESTOSTORE];
    int yval[NUMBEROFVALUESTOSTORE];
    bool xvalid[4];
    bool yvalid[4];
    int currentindex;
    int lastindex() { //reverse counting modulo
      return (((currentindex-1)<0) ? (NUMBEROFVALUESTOSTORE+currentindex-1) : (currentindex-1));
    }
    int secondtolastindex() {
      return (((currentindex-2)<0) ? (NUMBEROFVALUESTOSTORE+currentindex-2) : (currentindex-2));
    }
    int nextindex() {
      return (currentindex+1)%NUMBEROFVALUESTOSTORE;
    }
  public:
    FlexibleIntervalADNS3050():currentindex(0), lastupdatetime(0){
      for (int i=0;i<NUMBEROFVALUESTOSTORE;i++) {
        xvalid[i] = false;
        yvalid[i] = false;
      }
    }
    void update() {
      unsigned long currentupdatetime(micros());
      ADNS3050Reader::update();
      updatetimes[currentindex] = currentupdatetime-lastupdatetime;
      xval[currentindex]=last_x;
      yval[currentindex]=last_y;
      xvalid[currentindex] = (last_x<128)&&(last_x>-128);
      yvalid[currentindex] = (last_y<128)&&(last_y>-128);
      lastupdatetime = currentupdatetime;
      currentindex = nextindex();
    }
    bool lastvaluevalid() {
      return xvalid[lastindex()]&&yvalid[lastindex()];
    }
    float velocity_x(){
      return (float(xval[lastindex()])/float(updatetimes[lastindex()]))*(1e6/COUNTSPERMETER_DEFAULT);
    }
    float velocity_y(){
      return (float(yval[lastindex()])/float(updatetimes[lastindex()]))*(1e6/COUNTSPERMETER_DEFAULT);
    }
};

#endif
