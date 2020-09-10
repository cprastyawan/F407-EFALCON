/*
 * pid.c
 *
 *  Created on: Aug 18, 2020
 *      Author: LENOVO
 */

#include "pid.h"
#include "GlobalVariables.h"

void PIDControl(PIDType_t *pidtype, float input, float setPoint){
	pidtype->setPoint = setPoint;
	pidtype->error = pidtype->setPoint - input;

	pidtype->sumIntegral += pidtype->error * pidtype->timesampling;
	pidtype->sumIntegral = constrain(pidtype->sumIntegral, -2000, 2000);

	pidtype->derivative = (pidtype->error - pidtype->preverror) / pidtype->timesampling;
	pidtype->preverror = pidtype->error;

	pidtype->output = (pidtype->kp * pidtype->error) + (pidtype->kd * pidtype->derivative) + (pidtype->ki * pidtype->sumIntegral);
}

void PIDControlYAW(PIDType_t *pidtype){

}
void PIDControlROLL(PIDType_t *pidtype){

}
void PIDControlPITCH(PIDType_t *pidtype){

}
void PIDReset(PIDType_t *pidtype){
	pidtype->sumIntegral = 0;
	pidtype->output = 0;
}
void PIDInit(PIDType_t *pidtype, double kp, double ki, double kd, double timesampling){
	pidtype->kp = kp;
	pidtype->kd = kd;
	pidtype->ki = ki;

	pidtype->timesampling = timesampling;
}
void trustControl(){

}
