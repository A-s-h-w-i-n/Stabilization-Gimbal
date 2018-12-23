#include "mpu6050.h"
#include<Servo.h>

//float kp = 0.2, Kp = 0.2;
//float kd = 0.3, Kd = 0.3;
//float ki = 0.01, Ki = 0.01;
float kp = 0.2, Kp = 0.2;
float kd = 0, Kd = 0;
float ki = 0, Ki = 0;
float setPoint = 90, setPointPitch = 90;
float lastError = 0, lastErrorPitch = 0;
float errSum = 0, errSumPitch = 0;

Servo servoRoll, servoPitch;

void setup(){
  Serial.begin(115200);
  servoRoll.attach(6);
  servoPitch.attach(9);
  servoRoll.write(90);
  servoPitch.write(90);
  Serial.println("Enter any character to continue");
  while(1)
  {
    if(Serial.available())
    {      
      char buff = Serial.read();
      break;
    }
  }
  
  mpu6050.MPU6050Setup();
}

void loop(){

  float *YPR = mpu6050.MPU6050Loop();
  float feedBack, feedBackPitch;
  float err, errPitch;
  float errProportional, errDerivative, errIntegral;
  float PID_Out, PID_OutPitch;

  if(Serial.available()){
    char buff = Serial.read();
    switch(buff){
      case 'P': kp+=0.1; Kp+=0.1; Serial.print("kp = "); Serial.print(kp); Serial.print("Kp = "); Serial.println(Kp); break;
      case 'p': kp-=0.1; Kp-=0.1; Serial.print("kp = "); Serial.print(kp); Serial.print("Kp = "); Serial.println(Kp); break;
      case 'D': kd+=0.1; Kd+=0.1; Serial.print("kd = "); Serial.print(kd); Serial.print("Kd = "); Serial.println(Kd); break;
      case 'd': kd-=0.1; Kd-=0.1; Serial.print("kd = "); Serial.print(kd); Serial.print("Kd = "); Serial.println(Kd); break;
      case 'I': ki+=0.01; Ki+=0.01; Serial.print("ki = "); Serial.print(ki); Serial.print("Ki = "); Serial.println(Ki); break;
      case 'i': ki-=0.01; Ki-=0.01; Serial.print("ki = "); Serial.print(ki); Serial.print("Ki = "); Serial.println(Ki); break;
      case 'T': setPointPitch++; break;
      case 't': setPointPitch++; break;
      case 'G': setPointPitch--; break;
      case 'g': setPointPitch--; break;
      case 'H': setPoint++; break;
      case 'h': setPoint++; break;
      case 'F': setPoint--; break;
      case 'f': setPoint--; break;
      default: setPoint = 90; setPointPitch = 90; break;
    }
    buff = 'n';
  }
  
  /*Serial.print("Roll: ");
  Serial.print(YPR[1]);
  Serial.print("\tPitch: ");
  Serial.println(YPR[2]);*/
 
  feedBack = ((YPR[1]+0.83)/1.66)*180;
  //Serial.print(YPR[1]);
  //Serial.print("\t");
  //Serial.print(feedBack);

  err = setPoint - feedBack;

  //Serial.print("\t\tErrorRoll: ");
  //Serial.print(err);

  errProportional = kp*err;

  errDerivative = kd*(err - lastError);
  lastError = err;

  errSum += err;
  errIntegral = ki*errSum;

  //Serial.print(" ");
  //Serial.println(errIntegral);
  
  PID_Out = errProportional + errDerivative + errIntegral;

  //Serial.print("\t\tPID_Out: ");
  //Serial.println(PID_Out);

                      //Serial.print("Pitch: ");
                     
                      feedBackPitch = ((YPR[2]+0.83)/1.66)*180;
                      //Serial.print(YPR[2]);
                      //Serial.print("\t");
                      //Serial.print(feedBackPitch);
                    
                      errPitch = setPointPitch - feedBackPitch;
                    
                      //Serial.print("\t\tErrorPitch: ");
                      //Serial.print(errPitch);
                    
                      errProportional = Kp*errPitch;
                    
                      errDerivative = Kd*(errPitch - lastErrorPitch);
                      lastErrorPitch = errPitch;                    
                    
                      errSumPitch += errPitch;
                      errIntegral = Ki*errSumPitch;
                    
                      //Serial.print(" ");
                      //Serial.println(errIntegral);
                      
                      PID_OutPitch = errProportional + errDerivative + errIntegral;
                    
                      //Serial.print("\t\tPID_OutPitch: ");
                      //Serial.println(PID_OutPitch); 

  Serial.print("RollErr: ");
  Serial.print(err);
  Serial.print("\tPitchErr: ");
  Serial.println(errPitch);
  
  if(PID_Out > 0){
    increaseAngleRoll(PID_Out);
  }
  else if(PID_Out < 0){
    decreaseAngleRoll(PID_Out);
  } 

                        if(PID_OutPitch > 0){
                          increaseAnglePitch(PID_OutPitch);
                        }
                        else if(PID_OutPitch < 0){
                          decreaseAnglePitch(PID_OutPitch);
                        } 

  delay(20);
}

void increaseAngleRoll(float x){
  servoRoll.write(servoRoll.read() + fabs(x));
}

void decreaseAngleRoll(float x){
  servoRoll.write(servoRoll.read() - fabs(x));
}

                        void increaseAnglePitch(float x){
                          servoPitch.write(servoPitch.read() + fabs(x));
                        }
                        
                        void decreaseAnglePitch(float x){
                          servoPitch.write(servoPitch.read() - fabs(x));
                        }
