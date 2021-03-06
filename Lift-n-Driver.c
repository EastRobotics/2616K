#pragma config(Sensor, in1,    pot1,           sensorPotentiometer)
#pragma config(Motor,  port1,          	clawArmMR,     tmotorVex393HighSpeed_HBridge, openLoop)
#pragma config(Motor,  port2,           leftfront,      tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port3,           leftrear,     tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           clawArmTL,     tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           clawArmTR,     tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           claw,     tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           clawArmB,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           rightrear,          tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           rightfront,    tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port10,          clawArmML,     tmotorVex393HighSpeed_HBridge, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#pragma platform(VEX2)
#pragma competitionControl(Competition)
#include "Vex_Competition_Includes.c"
#define FL motor[leftfront]
#define FR motor[rightfront]
#define BR motor[rightrear]
#define BL motor[leftrear]

task Drive() {
	while(true) {
		switch(vexRT[Btn5U]) {
		case 1:
			/*
			motor[leftfront] = vexRT[Ch3] + vexRT[Ch1] + vexRT[Ch4];
			motor[rightfront] = vexRT[Ch3] - vexRT[Ch1] - vexRT[Ch4];
			motor[leftrear] = vexRT[Ch3] + vexRT[Ch1] - vexRT[Ch4];
			motor[rightrear]= vexRT[Ch3] - vexRT[Ch1] + vexRT[Ch4];*/
			FL = -(vexRT[Ch3] /* Y */ + vexRT[Ch4] /* X */) - vexRT[Ch1] /* R */;
				/* FL = -(y + x) - r */
			FR = -(-vexRT[Ch3 /* Y */ + vexRT[Ch4] /* X */) - vexRT[Ch1] /* R */;
				/* FR = -(-y + x) - r */
			BR = -(vexRT[Ch3] /* Y */ - vexRT[Ch4] /* X */) - vexRT[Ch1] /* R */;
				/* BR = -(y + x) - r */
			BL = -(vexRT[Ch3] /* Y */ - vexRT[Ch4] /* X */) - vexRT[Ch1] /* R */;
				/* BL = -(y - x) - r */
			break;
		case 0:

			break;
		}
	}
}
task clawArm() {
	while(true){
		switch(vexRT[Btn6U]) {
		case 1:
			motor[clawArmB] = -127;
			motor[clawArmML] = -127;
			motor[clawArmTL] = -127;
			motor[clawArmMR]= -127;
			motor[clawArmTR] = -127;
			break;
		case 0:
			motor[clawArmB] = 0;
			motor[clawArmML] = 0;
			motor[clawArmTL] = 0;
			motor[clawArmMR]= 0;
			motor[clawArmTR] = 0;
			break;
		}
		switch(vexRT[Btn6D]) {
		case 1:
			motor[clawArmB] = 127;
			motor[clawArmML] = 127;
			motor[clawArmTL] = 127;
			motor[clawArmMR]= 127;
			motor[clawArmTR] = 127;
			break;
		case 0:
			motor[clawArmB] = 0;
			motor[clawArmML] = 0;
			motor[clawArmTL] = 0;
			motor[clawArmMR]= 0;
			motor[clawArmTR] = 0;
			break;
		}
	}
}

task clawGrasp() {
	while(true) {
		switch(vexRT[Btn5U]) {
		case 1:
			motor[claw] = 127;
			while(vexRT[Btn5U]){

			}
		case 0:
			motor[claw] = 0;
		}

		switch(vexRT[Btn5D]) {
		case 1:
			motor[claw] = -127;
			while(vexRT[Btn5D]){

			}
			break;
		case 0:
			motor[claw] = 0;
			break;
		}
	}
}

void pre_auton() {
	bStopTasksBetweenModes = true;
}

task autonomous() {

}

task usercontrol() {
	startTask(Drive);
	startTask(clawArm);
	startTask(clawGrasp);
}
