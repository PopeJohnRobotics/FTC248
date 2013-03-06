/*
Copyright (c) 2013 Pope John XXIII High School Robotics Team

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in the
Software without restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*
#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTServo,  none)
#pragma config(Sensor, S2,     gyro,          sensorI2CCustomFastSkipStates)
#pragma config(Sensor, S3,     angle,          sensorI2CCustomFastSkipStates)
#pragma config(Sensor, S4,     irsensor,			sensorI2CCustomFastSkipStates)
#pragma config(Servo,  srvo_S1_C3_1,    servo1,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_2,    rampGate,             tServoStandard)
#pragma config(Servo,  srvo_S1_C3_3,    gripper,              tServoStandard)
#pragma config(Servo,  srvo_S1_C3_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_5,    rampLaunchR,          tServoStandard)
#pragma config(Servo,  srvo_S1_C3_6,    rampLaunchL,          tServoStandard)
*/

#include "Driver Suite/common.h"
#include "Driver Suite/hitechnic-angle.h"
#include "Driver Suite/hitechnic-irseeker-v2.h"
#include "JoystickDriver.c"

#define left mtr_S1_C1_2
#define right mtr_S1_C1_1
#define arm mtr_S1_C2_1
#define lift mtr_S1_C2_2
/*#define rampGate 13
#define gripper 14
#define rampLaunchR 16
#define rampLaunchL 17
*/
//#define angle S3
//#define irsensor S2



void rightSide(int power){
	motor[right] = -power;
}
void leftSide(int power){
	motor[left] = power;
}

long armTarget;
bool armEnable;
bool armHold;
long I;

void setArmTarget(int target){
	hogCPU();
	armTarget = min2(0,max2(-800,target));
	releaseCPU();
}

void enableArm(){
	hogCPU();
	armEnable = true;
	releaseCPU();
}

void disableArm(){
	hogCPU();
	armEnable = false;
	motor[arm] = 0;
	releaseCPU();
}

void holdArm(){
	hogCPU();
	armHold = true;
	I = 0;
	releaseCPU();
}

void unHoldArm(){
	hogCPU();
	armHold = false;
	I = 0;
	releaseCPU();
}

void setArmRunning(bool value){
	hogCPU();
	armEnable = value;
	releaseCPU();
}

long armPos(){
	return HTANGreadAccumulatedAngle(angle);
}

task armVirtualServo(){
	float kP,kI,kD;
	long oldP;
	kP = 1.0;
	kI = 0.01;
	kD = 0.20;
	armTarget = armPos();
	while (true){
		long p = armPos()-armTarget;
		int set = p*kP+(p-oldP)*kD+(I+=(abs(p)>2?p:0))*kI;
		if (armEnable) motor[arm] = (abs(p)>2 && !armHold)?set:0;
		if (oldP < 0 && 0 < p) I = 0;
		//writeDebugStreamLine("pos= %d, p= %d, i= %d, d= %d",armPos(),p,I,p-oldP);
		oldP = p;
		I = max2(1500,min2(-1500,I));
		wait1Msec(5);
		EndTimeSlice();
	}
}

///////////////////////////////////////////////////////////
// CONFIG
///////////////////////////////////////////////////////////
int gyroOff = 600;// Usually correct.
#define HAS_GYRO 1

int libftc_initGyro(){
  int error = 0;
  if (HAS_GYRO){
    gyroOff = 0;
    int min = 9001,max = 0;
    //PlayTone(7000,10);
    for (int i = 0; i < 10; ++i){
      int sens_Val;
      gyroOff += (sens_Val = SensorRaw[gyro]);
      sens_Val < min ? min = sens_Val : (sens_Val > max ? max = sens_Val : 1);// Set the max and min values.
    }
    //PlayTone(2000,10);
    gyroOff /= 10;
    if (gyroOff < 400){ error++; writeDebugStreamLine("gyroOff too low.");    }
    if (gyroOff > 800){ error++; writeDebugStreamLine("gyroOff too high.");   }
    if (max - min > 5){ error++; writeDebugStreamLine("Max-min: %d",max-min); }
    nxtDisplayCenteredTextLine(1,"%d",gyroOff);
  }
  return error;
}

void turnLeftForGyroAndStop(int power,int gyroDist)
{

  rightSide(power);
  leftSide(-power);
  float gyroVal = 0;
  bool running = true;
  bool returning = false;
  ClearTimer(T3);
  while (running) {
  	ClearTimer(T1);
    float deltaGyro = (float)(abs(SensorValue[gyro]-gyroOff) < 2 ? 0 : (SensorValue[gyro]-gyroOff))/200.0;
    gyroVal += deltaGyro;
    if (abs(gyroVal) > abs(gyroDist)){
      returning = true;
    }
    if (returning){
      rightSide(-power/3);
      leftSide(power/3);
      if (abs(gyroVal) < abs(gyroDist)){
        running = false;
        writeDebugStreamLine("Final GyroVal: %f",gyroVal);
      }
    }
    if (time1[T3] > 10000) running = false;
		//writeDebugStreamLine("%f",gyroVal);
    wait1Msec(5-time1[T1]);
  }
  rightSide(0);
  leftSide(0);
}

void forwardForTimeAndStop(int power,int time){
	rightSide(power);
	leftSide(power);
	wait1Msec(time);
	rightSide(0);
	leftSide(0);
}

void turnRightForTimeAndStop(int power,int time){
	rightSide(-power);
	leftSide(power);
	wait1Msec(time);
	rightSide(0);
	leftSide(0);
}

void initializeRobot()
{
	I = 0;
	StopTask(armVirtualServo);
	servo[gripper] = 0;
	servo[rampGate] = 100;
	servo[rampLaunchL] = 255;
	servo[rampLaunchR] = 0;
	disableArm();
	unHoldArm();
	StartTask(armVirtualServo,kDefaultTaskPriority-2);
	libftc_initGyro();
  return;
}
