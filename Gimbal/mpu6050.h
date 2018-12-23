#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include<Wire.h>
#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2  
#define LED_PIN 13

class MPU6050_New {

private:
  
      MPU6050 mpu;
      bool blinkState = false;
      bool dmpReady = false;  // set true if DMP init was successful
      uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
      uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
      uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
      uint16_t fifoCount;     // count of all bytes currently in FIFO
      uint8_t fifoBuffer[64]; // FIFO storage buffer

      Quaternion q;          
      VectorInt16 aa;         
      VectorInt16 aaReal;     
      VectorInt16 aaWorld;    
      VectorFloat gravity;    
      float euler[3];         

      uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

      volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
      
      void dmpDataReady() {
      mpuInterrupt = true;
      }

public:
      float ypr[3];        
      
      void MPU6050Setup(){
            
          #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
              Wire.begin();
              Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
          #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
              Fastwire::setup(400, true);
          #endif
                
          mpu.initialize();
          pinMode(INTERRUPT_PIN, INPUT);
      
          delay(1000);
        
          devStatus = mpu.dmpInitialize();
      
          mpu.setXGyroOffset(-8);
          mpu.setYGyroOffset(9);
          mpu.setZGyroOffset(-8);
          mpu.setZAccelOffset(2107);
          // make sure it worked (returns 0 if so)
          if (devStatus == 0) {
              mpu.setDMPEnabled(true);
      
              mpuIntStatus = mpu.getIntStatus();
      
              dmpReady = true;
      
              // get expected DMP packet size for later comparison
              packetSize = mpu.dmpGetFIFOPacketSize();
              pinMode(LED_PIN, OUTPUT);
          } 
      }
      
      float *MPU6050Loop(){         
    
          if (!dmpReady) return;
      
          mpuInterrupt = false;
          mpuIntStatus = mpu.getIntStatus();
      
          // get current FIFO count
          fifoCount = mpu.getFIFOCount();
          mpu.resetFIFO();
          fifoCount = mpu.getFIFOCount();
      
          if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
              mpu.resetFIFO();
              Serial.println(F("FIFO overflow!"));
      
          } else if (mpuIntStatus & 0x02) {
              while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      
              mpu.getFIFOBytes(fifoBuffer, packetSize);
      
              fifoCount -= packetSize;
      
              mpu.dmpGetQuaternion(&q, fifoBuffer);
              mpu.dmpGetGravity(&gravity, &q);
              mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
          }    
          
          mpu.resetFIFO();
          
          return ypr;
      }
} mpu6050;
