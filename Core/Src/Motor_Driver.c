/*
 * Motor_Driver.c
 *
 *  Created on: Jul 4, 2021
 *      Author: VIKI
 */


#include "Motor_Driver.h"

JCU_Config_t JCUConfig;
JCU_State_t JCUState;

// creating pointers to the motor structures
uint16_t *pJCUConfig = (uint16_t*) &JCUConfig;
uint16_t *pJCUState = (uint16_t*) &JCUState;


float tau; 												// Derivative time constant
volatile uint16_t adcResultDMA[3];
uint16_t PWMValue = 500;
uint32_t PreviousEncoderCount = 0;
uint16_t counter = 0;
uint32_t EncoderCounter = 0;
float PosError;
float prevPosError;

void UpdatePWM(void)
{
	HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);


	/*
	 * position loop
	 */

	/*
	// 1. Measuring error position
	PosError = JCUConfig.TargetAngel - JCUState.Angle;

	// 2. Proportional
	float Proportional = JCUConfig.KpPossitionLoop * PosError;
	float Integral = 0;
	float Derivative = 0;
	float PID = Proportional + Integral + Derivative;
	if (PID >= 0)
	{
		PID = 500 + PID;
		if (PID < MAX_DUTY_CYCLE)
			PWMValue = PID;
		else
			PID = MAX_DUTY_CYCLE;
	}
	else
	{
		PID = 500 + PID;
		if (PID > MIN_DUTY_CYCLE)
			PWMValue = PID;
		else
			PID = MIN_DUTY_CYCLE;
	}
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000 - PWMValue);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWMValue);
	*/
}


void RunMotor(void)
{
	/*
	 * We calling this function with 40 kHz
	 * due to dummy cycles, we read encoder data and analog data with 10 kHz each
	 */
	if (FeedbackState == READ_ENCODER)
	{
		EncoderRoutine();
		FeedbackState = WAIT_ENCODER;
	}
	else if(FeedbackState == WAIT_ENCODER)
	{
		//dummy step
		FeedbackState = READ_ANALOG;
	}
	else if (FeedbackState == READ_ANALOG)
	{
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcResultDMA, 3);
		FeedbackState = WAIT_ANALOG;
	}
	else if(FeedbackState == WAIT_ANALOG)
	{
		//dummy step
		FeedbackState = READ_ENCODER;
		JCUState.Speed = EncoderCounter - PreviousEncoderCount;
		PreviousEncoderCount = EncoderCounter;
	}

	if (MotorState == MOTOR_RUN)
	{
		UpdatePWM();
	}

}

// Count increment encoder ticks
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	EncoderCounter = __HAL_TIM_GET_COUNTER(htim);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	RunMotor();
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{

}


void CheckControlRegister(void)
{
	// Check bits in status register ONLY which could be written by Master.
	if 	(CHECK_BIT(JCUConfig.ControlRegister, ENABLE_MOTOR_Pos))
	{
		if (MotorState == MOTOR_DISABLED)			// enable motor if it is not enabled
		{
			MotorState = MOTOR_ENABLED;
			EnableMotor();
		}
	}
	else
	{
		if (MotorState != MOTOR_DISABLED)
		{
			MotorState = MOTOR_DISABLED;
			DisableMotor();
		}
	}


	if (CHECK_BIT(JCUConfig.ControlRegister, SET_BRAKE_Pos))
	{
		// apply brake only if motor is not running
		if(MotorState != MOTOR_RUN)
		{
			// TO DO: Apply brake, obviously
		}
	}
	else
	{
		// should  i reset brake??
	}


	if (CHECK_BIT(JCUConfig.ControlRegister, GO_TO_TARGET_POSITION_Pos))
	{
		if (MotorState == MOTOR_ENABLED)
		{
			MotorState = MOTOR_RUN;
		}

	}
	else
	{
		// decide what to do here
	}

	if (CHECK_BIT(JCUConfig.ControlRegister, STOP_MOTOR_Pos))
	{
		JCUConfig.ControlRegister &=~ GO_TO_TARGET_POSITION;				// not go to target position anymore
		// 1. Terminate PID
		MotorState = MOTOR_ENABLED;
		// 2. set PWM 50%
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 500);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 500);
	}

}

void EnableMotor(void)
{

	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start_IT(&htim1, TIM_CHANNEL_2);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 500);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 500);
	MotorState = MOTOR_ENABLED;
}

void DisableMotor(void)
{
	HAL_TIM_Base_Stop_IT(&htim1);
	HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Stop_IT(&htim1, TIM_CHANNEL_2);
	MotorState = MOTOR_DISABLED;
}

/*
void TestAsignment(void)
{

	JCUConfig.Acceleration = 140;
	JCUConfig.Deceleration = 160;
	JCUConfig.MaxSpeed = 200;
	JCUConfig.MaxTorque = 6;
	JCUConfig.TargetAngel = 16000;
	JCUConfig.ControlRegister = 50000;
	JCUConfig.KpCurrentLoop = 152.25;
	JCUConfig.KiCurrentLoop = -18.5;
	JCUConfig.KdCurrentLoop = 32;
	JCUConfig.KpPossitionLoop = 1.8995;
	JCUConfig.KiPossitionLoop = 5546;
	JCUConfig.KdPossitionLoop = 10000;
	JCUConfig.KpSpeedLoop = 2000000;
	JCUConfig.KiSpeedLoop = 31;
	JCUConfig.KdSpeedLoop = 4;

	JCUState.Errors = 530;
	JCUState.Angle = 41000;
	JCUState.Speed = -358;
	JCUState.Torque = 400;
	JCUState.MotorTemp = 40;
	JCUState.HbridgeTemp = 35;
}
*/
