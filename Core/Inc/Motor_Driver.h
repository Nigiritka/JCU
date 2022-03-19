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



/*
 * DEFINES STATUS Register
 */

#define ENABLE_MOTOR_Pos					(0U)									// Enable timer which control PWM for the motor
#define ENABLE_MOTOR_Msk					(0x1UL << ENABLE_MOTOR_Pos)				/*!< 0x00000001 */
#define ENABLE_MOTOR						ENABLE_MOTOR_Msk

#define SET_BRAKE_Pos						(1U)									// Enable Brake
#define SET_BRAKE_Msk						(0x1UL << SET_BRAKE_Pos)				/*!< 0x00000010 */
#define SET_BRAKE							SET_BRAKE_Msk

#define BRAKE_STATUS_Pos					(2U)									// this bit is set if brake is set
#define BRAKE_STATUS_Msk					(0x1UL << BRAKE_STATUS_Pos)				/*!< 0x00000100 */
#define BRAKE_STATUS						BRAKE_STATUS_Msk

#define GO_TO_TARGET_POSITION_Pos			(3U)									// Motor will go to target position which is set in JCU status structure
#define GO_TO_TARGET_POSITION_Msk			(0x1UL << GO_TO_TARGET_POSITION_Pos)	/*!< 0x00001000 */
#define GO_TO_TARGET_POSITION				GO_TO_TARGET_POSITION_Msk

#define REACHED_TARGET_POSITION_Pos			(4)										// FLAG if motor reached target position
#define REACHED_TARGET_POSITION_Msk			(0x1UL << REACHED_TARGET_POSITION_Pos)	/*!< 0x00010000 */
#define REACHED_TARGET_POSITION				REACHED_TARGET_POSITION_Msk

#define STOP_MOTOR_Pos						(5)										// Stop motor in current position, PWM still will be apply 50%
#define STOP_MOTOR_Msk						(0x1UL << STOP_MOTOR_Pos)				/*!< 0x00010000 */
#define STOP_MOTOR							STOP_MOTOR_Msk


/*
 * DEFINES motor parameters
 */

#define MAX_DUTY_CYCLE						900
#define MIN_DUTY_CYCLE						100

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


/*
 * Motor State machine
 */

typedef enum
{
	MOTOR_DISABLED = 0U,			// remove these 0 1 2
	MOTOR_ENABLED = 1U,				// 50% duty cycle, does not move
	MOTOR_RUN = 2U,

} eMotorStateMachine;

eMotorStateMachine MotorState;

void CheckStatusRegister(void);
void TestAsignment(void);
void RunMotor(void);
void EnableMotor(void);
void DisableMotor(void);
void UpdatePWM(void);
int16_t SpeedCalculation(void);

#endif /* INC_MOTOR_DRIVER_H_ */

