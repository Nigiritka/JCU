/*
 * Motor_Driver.h
 *
 *  Created on: Jul 4, 2021
 *      Author: VIKI
 */

#ifndef INC_MOTOR_DRIVER_H_
#define INC_MOTOR_DRIVER_H_

#include "main.h"
#include "JCU_defines.h"


extern TIM_HandleTypeDef htim1;
extern ADC_HandleTypeDef hadc1;


/*
 * DEFINES motor parameters
 */

#define MAX_DUTY_CYCLE						900
#define MIN_DUTY_CYCLE						100

/*
 * Make a structure with all this JCU parameters
 */



/*
 * Motor State machine
 */

typedef enum
{
	MOTOR_DISABLED,
	MOTOR_ENABLED,				// 50% duty cycle, does not move
	MOTOR_RUN,					// Go target position
	MOTOR_STAY					// Keep current position, which was at the moment of activation of this mode

} eMotorStateMachine;

eMotorStateMachine MotorState;

typedef enum
{
	READ_ENCODER,
	WAIT_ENCODER,
	READ_ANALOG,
	WAIT_ANALOG,

} eFeedbackStateMachine;

eFeedbackStateMachine FeedbackState;

void CheckControlRegister(void);
void TestAsignment(void);
void RunMotor(void);
void EnableMotor(void);
void DisableMotor(void);
void UpdatePWM(void);
void test(void);

#endif /* INC_MOTOR_DRIVER_H_ */

