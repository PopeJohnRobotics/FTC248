#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTServo,  none)
#pragma config(Sensor, S2,     gyro,           sensorI2CHiTechnicGyro)
#pragma config(Sensor, S3,     angle,          sensorI2CCustomFastSkipStates)
#pragma config(Sensor, S4,     irsensor,       sensorI2CCustomFastSkipStates)
#pragma config(Motor,  mtr_S1_C1_1,     motorD,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     motorE,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     motorF,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     motorG,        tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C3_1,    servo1,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_2,    rampGate,             tServoStandard)
#pragma config(Servo,  srvo_S1_C3_3,    gripper,              tServoStandard)
#pragma config(Servo,  srvo_S1_C3_4,    rampLaunchL,          tServoStandard)
#pragma config(Servo,  srvo_S1_C3_5,    rampLaunchR,          tServoStandard)
#pragma config(Servo,  srvo_S1_C3_6,    servo6,               tServoStandard)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

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

//#include "Driver Suite/driverz/common.h"
//#include "JoystickDriver.c"  //Include file to "handle" the Bluetooth messages.
#include "248header.h"

#define fabs(x) (x<0?-x:x)

float joystickTransform(int i){
	float v = i/127.0;
	return fabs(v) < 0.05?0:v*v*v;
}

task main()
{
	writeDebugStreamLine("BAT: %d",externalBatteryAvg);
	initializeRobot();
	bFloatDuringInactiveMotorPWM = false;
	waitForStart();   // wait for start of tele-op phase
	setArmTarget(-200);
	I = 0;
	enableArm();
	int gripperPos = 0; // Location of our gripper
	int armSpeed = 0; // Speed target on the arm
	long armTarg = -200; // Position target on the arm
	int nudgeClamp = 6; // Allows nudges on the arm
	//int rampPos = 100; // Old starting position for the ramp servos
	int rampLaunchPos = 240; // Starting position for ramp servos
	int oldHat = -1; // Detect POVhat transitions
	int armOffset = 0; // Kludge to make presets work
	while (true){
		getJoystickSettings(joystick);

		// Drivetrain code.
		rightSide(100*joystickTransform(joystick.joy1_y2));
		leftSide(100*joystickTransform(joystick.joy1_y1));

		// armEnable controls whether we use PID control (true) or old style control (false).
		if (!armEnable){
			int powerMode = joy2Btn(3)?-80:-50;
			int setPoint = joy2Btn(6)?powerMode:(joy2Btn(8)?-powerMode:(joystick.joy2_TopHat == -1?-10:0));
			armSpeed = setPoint;
			motor[arm] = joy2Btn(2)? 80*(nudgeClamp>0?nudgeClamp--:nudgeClamp=0):
									(joy2Btn(4)?-80*(nudgeClamp>0?nudgeClamp--:nudgeClamp=0):armSpeed);

			if (!joy2Btn(4) && !joy2Btn(2)) nudgeClamp = 6;
		}else{
			if (joystick.joy2_TopHat != -1 && oldHat == -1) holdArm();
			if (joystick.joy2_TopHat == -1 && oldHat != -1) unHoldArm();

			if (joy2Btn(10)){
				armOffset += joy2Btn(6)?1:(joy2Btn(8)?-1:0);
				armTarg +=   joy2Btn(6)?1:(joy2Btn(8)?-1:0);
			} else armTarg += joy2Btn(6)?3:(joy2Btn(8)?-3:0);

			armTarg = min2(0,max2(-800,armTarg));

			// Arm presets
			if (joy2Btn(1)) armTarg = -720+armOffset;
			if (joy2Btn(2)) armTarg = -620+armOffset;
			if (joy2Btn(3)) armTarg = -560+armOffset;
			if (joy2Btn(4)) armTarg = -315+armOffset;
			setArmTarget(armTarg);
		}

		if (joy2Btn(2)&&joy2Btn(4)&&joy2Btn(3)){
			setArmRunning(!armEnable);
		}

		motor[lift] = joy1Btn(6)?90:(joy1Btn(8)?-90:0);

		if (joy2Btn(1)) gripperPos = 78;
		if (joy2Btn(4)) gripperPos = 240;

		servo[gripper] = joy2Btn(5)?++gripperPos:(joy2Btn(7)?--gripperPos:gripperPos);

		gripperPos = max2(0,min2(255,gripperPos));

		rampLaunchPos = joy1Btn(9)&&joy1Btn(10)?45:((joy1Btn(1)&&joy1Btn(2))?240:rampLaunchPos);

		servo[rampLaunchR] = (1.0-(rampLaunchPos/255.0))*255;
		servo[rampLaunchL] = rampLaunchPos;

		// End-of-loop stuff
		oldHat = joystick.joy2_TopHat;
		int t = (time1[T1]>15?14:time1[T1]);
		wait1Msec(15-t);
		ClearTimer(T1);
	}
}
