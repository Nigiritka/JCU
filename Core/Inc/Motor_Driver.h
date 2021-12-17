/*
 * Motor_Driver.h
 *
 *  Created on: Jul 4, 2021
 *      Author: VIKI
 */

#ifndef INC_MOTOR_DRIVER_H_
#define INC_MOTOR_DRIVER_H_

#include "main.h"
#include "AS5048A.h"

extern TIM_HandleTypeDef htim1;
extern ADC_HandleTypeDef hadc1;

#define WRITE_MULTIPLE_AOHR_BYTES_RESPONSE			8

/*
 * Make a structure with all this JCU parameters
 */

typedef struct
{
	uint8_t Acceleration;
	uint8_t Deceleration;
	uint8_t MaxSpeed;
	uint8_t MaxTorque;
	uint16_t TargetAngel;
	uint16_t StatusRegister;
	float KpCurrentLoop;
	float KiCurrentLoop;
	float KdCurrentLoop;
	float KpPossitionLoop;
	float KiPossitionLoop;
	float KdPossitionLoop;
	float KpSpeedLoop;
	float KiSpeedLoop;
	float KdSpeedLoop;

}JCU_Config_t;

typedef struct
{
	uint16_t Errors;
	uint16_t Angle;
	int16_t Speed;
	int16_t Torque;
	uint8_t MotorTemp;
	uint8_t HbridgeTemp;

}JCU_State_t;

JCU_Config_t JCUConfig;

JCU_State_t JCUState;



void TestAsignment(void);
void RunMotor(void);
void EnableMotor(void);
void DisableMotor(void);

#endif /* INC_MOTOR_DRIVER_H_ */

