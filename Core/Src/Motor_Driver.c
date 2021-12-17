/*
 * Motor_Driver.c
 *
 *  Created on: Jul 4, 2021
 *      Author: VIKI
 */


#include "Motor_Driver.h"

// creating pointers to the motor structures
uint16_t *pJCUConfig = (uint16_t*) &JCUConfig;
uint16_t *pJCUState = (uint16_t*) &JCUState;

uint8_t PeriodCounter=0;



void RunMotor(void)
{

	//EnableAlarmLED();

	if (PeriodCounter == 0)
	{
		ReadAngle_RequestErrors();
		PeriodCounter++;
	}
	else
	{
		ReadErrors_RequestAngle();
		HAL_ADC_Start_IT(&hadc1);
		PeriodCounter=0;
	}

/*
	uint16_t DutyCycle = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1);

	switch (PeriodCounter)
	{
	case 0:
		if (DutyCycle >= 500)
		{
			//EnableAlarmLED();
			HAL_ADC_Start_IT(&hadc1);
			ReadAngle_RequestErrors();
		}
		break;
	case 1:
		if (DutyCycle < 500)
		{
			//EnableAlarmLED();
			HAL_ADC_Start_IT(&hadc1);
			ReadAngle_RequestErrors();
		}
		break;
	case 2:
		if (DutyCycle >= 500)
		{
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, JCUState.Torque/4);
			ReadErrors_RequestAngle();
		}
		break;
	case 3:
		if (DutyCycle < 500)
		{
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, JCUState.Torque/4);
			ReadErrors_RequestAngle();
		}
		break;
	}


	if (PeriodCounter < 3)
		PeriodCounter++;
	else
		PeriodCounter = 0;
*/

}

void EnableMotor(void)
{

	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start_IT(&htim1, TIM_CHANNEL_2);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 400);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 600);
	//JCUConfig.StatusRegister |= ENABLE_MOTOR;

	//ClearErrorFlags();
}

void DisableMotor(void)
{
	HAL_TIM_Base_Stop_IT(&htim1);
	HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Stop_IT(&htim1, TIM_CHANNEL_2);
	JCUConfig.StatusRegister &=~ ENABLE_MOTOR;
}

/*
void TestAsignment(void)
{

	JCUConfig.Acceleration = 140;
	JCUConfig.Deceleration = 160;
	JCUConfig.MaxSpeed = 200;
	JCUConfig.MaxTorque = 6;
	JCUConfig.TargetAngel = 16000;
	JCUConfig.StatusRegister = 50000;
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
