#pragma config(Sensor, in3,    Gyro,           sensorGyro)
#pragma config(Sensor, in6,    LiftPotR,       sensorPotentiometer)
#pragma config(Sensor, in7,    LiftPotL,       sensorPotentiometer)
#pragma config(Sensor, in8,    MogoPot,        sensorPotentiometer)
#pragma config(Sensor, dgtl9,  quadLeft,       sensorQuadEncoder)
#pragma config(Sensor, dgtl11, quadRight,      sensorQuadEncoder)
#pragma config(Motor,  port1,           right1,        tmotorVex393HighSpeed_HBridge, openLoop, reversed, driveRight, encoderPort, dgtl11)
#pragma config(Motor,  port2,           right2,        tmotorVex393HighSpeed_MC29, openLoop, reversed, driveRight)
#pragma config(Motor,  port3,           liftright,     tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port4,           liftleft,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           claw,          tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           chain,         tmotorVex393_MC29, openLoop, encoderPort, None)
#pragma config(Motor,  port7,           left1,         tmotorVex393HighSpeed_MC29, PIDControl, driveLeft, encoderPort, dgtl9)
#pragma config(Motor,  port8,           left2,         tmotorVex393HighSpeed_MC29, openLoop, driveLeft)
#pragma config(Motor,  port9,           mogo,          tmotorVex393_MC29, openLoop)

float degrees;
float DriveTarget;
float GyroIntegral;
float DriveIntegral;

task gyroPIDturn(){

float turntarget;
float GyroCurrentPosition;
float GyroPidDifference;
float turningPower;
float GyroDerivative;
float lastGyroPidDifference;
float Gyro_Kp = 1;//constants
float Gyro_Ki = 0.0000000004;
float Gyro_Kd = 4;
turntarget = degrees*10;
lastGyroPidDifference = 0;
//GyroIntegral = 0;
while(true){
GyroCurrentPosition = SensorValue(Gyro);
GyroPidDifference = turntarget-GyroCurrentPosition; //creates error for proportional

GyroIntegral += GyroPidDifference; //creates integral

GyroDerivative = GyroPidDifference - lastGyroPidDifference; //creates derivative
lastGyroPidDifference = GyroPidDifference;

turningPower = GyroPidDifference*Gyro_Kp + GyroIntegral*Gyro_Ki + GyroDerivative*Gyro_Kd; //adds everything up
if(turningPower > 127) //limits motor power to scale
				turningPower = 127;
if(turningPower < -127)
				turningPower = -127;
motor[left1] = -turningPower; //sending power to the wheels
motor[left2] = -turningPower;
motor[right1] = turningPower;
motor[right2] = turningPower;
}
}


task drivePID(){
float CurrentEncoderPosition;
float DriveDifference;
float DriveDerivative;
float DrivePower;
float LastDriveDifference;
float Drive_Kp = 1.7;
float Drive_Ki = 0;
float Drive_Kd = 0;

int error;
int DriveST_Kp
int MasterPower = DrivePower;
int SlavePower = DrivePower;
SensorValue[quadLeft] = SensorValue[quadRight] = 0;
LastDriveDifference = 0;
while(true){
CurrentEncoderPosition = SensorValue[quadLeft];
DriveDifference = DriveTarget - CurrentEncoderPosition;

DriveIntegral += DriveDifference;

DriveDerivative = DriveDifference - LastDriveDifference;
LastDriveDifference = DriveDifference;

DrivePower = DriveDifference*Drive_Kp + DriveIntegral*Drive_Ki + DriveDerivative*Drive_Kd;
if(DrivePower > 127)
		DrivePower = 100;
if(DrivePower < -127)
		DrivePower = -100;
error = SensorValue[quadLeft] - SensorValue[quadRight];
SlavePower += error*DriveST_Kp;


motor[left1] = MasterPower; //sending power to the wheels
motor[left2] = MasterPower;
motor[right1] = SlavePower;
motor[right2] = SlavePower;
wait1Msec(100);

}
}

task main()
{

  while(SensorValue(MogoPot) > 1100)
  	{
  	motor[mogo] = -127;
	}
		motor[mogo] = 0;
	wait1Msec(500);
	DriveTarget = 360;
  startTask(drivePID);
  wait1Msec(1000);
  stopTask(drivePID);
  wait1Msec(1000);
  motor[left1] = 0; //stops the wheels
  motor[left2] = 0;
  motor[right1] = 0;
  motor[right2] = 0;
  while(SensorValue(MogoPot)< 3100)
  	{
  	motor[mogo] = 127;
	}
		motor[mogo] = 0;

	SensorValue(Gyro) = 0; //used to calibrate gyro at first
  SensorType[in1] = sensorNone;
  wait1Msec(1000);
  SensorType[in1] = sensorGyro;
  wait1Msec(2000);
  degrees = 180;
  startTask(gyroPIDturn);
  wait1Msec(1900);
  stopTask(gyroPIDturn);
  SensorValue(quadLeft) = SensorValue(quadRight) = 0;



}
