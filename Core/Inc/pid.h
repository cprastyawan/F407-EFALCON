/*
 * pid.h
 *
 *  Created on: Aug 18, 2020
 *      Author: LENOVO
 */

#ifndef INC_PID_H_
#define INC_PID_H_

typedef struct {
	float error;
	float preverror;

	float derivative;
	float sumIntegral;
	float setPoint;

	float output;

	float kp;
	float kd;
	float ki;

	float timesampling;
} PIDType_t;

void PIDControlYAW(PIDType_t *pidtype);
void PIDControlROLL(PIDType_t *pidtype);
void PIDControlPITCH(PIDType_t *pidtype);
void PIDReset(PIDType_t *pidtype);
void PIDInit(PIDType_t *pidtype, double kp, double ki, double kd, double timesampling);
void PIDControl(PIDType_t *pidtype);
void trustControl();

#endif /* INC_PID_H_ */
