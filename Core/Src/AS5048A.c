/*
 * AS5048A.c
 *
 *  Created on: Dec 21, 2020
 *      Author: VIKI
 */

#include "AS5048A.h"
#include "Motor_Driver.h"

uint8_t ReadAngle[SIZE] = {0xFF, 0xFF};							// Command to read angle
uint8_t Angle[SIZE] = {0};										// Variable for angle value
uint8_t NOP[SIZE] = {0x00, 0x00};								// Dummy reading
uint8_t ReadErrorFlagsAddress [SIZE] = {0x40, 0x01};			// Error register. All errors are cleared by access
uint8_t ZeroPositionRegHIAddress [SIZE] = {0x80, 0x16};			// address of the Hi zero position register of the AS5048
uint8_t ZeroPositionRegLOAddress [SIZE] = {0x00, 0x17};			// address of the Lo zero position register of the AS5048
uint8_t AGCDiagnosticAddress [SIZE] = {0x7F, 0xFD};				// address of the Diagnostics + Automatic Gain Control (AGC)
uint8_t AGCDiagnosticValue [SIZE] = {0};						// Variable for AGC value
uint8_t ProgRegWrite [SIZE] = {0x00, 0x03};						// Command for getting writing access to programming control register
uint8_t ProgRegValue [SIZE] = {0x80, 0x01};						// Enable AS5048 programming
uint16_t AngularPosition = 0;									// 16-bit angle for the JCU
uint8_t EncoderState = 0;


volatile bool comp_high = 0;									// if 1 - weak magnetic field
volatile bool comp_low = 0;										// if 1 - high magnetic field
volatile bool COF = 0;											// if 1 - invalid angle data, AS5048 internal error
volatile bool OCF = 1;											// always must be 1, if isn't - AS5048 internal error.
volatile bool EncoderErrorFlag = 0;

void CheckErrorsEnc()
{
	comp_high = (AGCDiagnosticValue[0] & 0x08);					// bitmask for comp_high flag
	comp_low = (AGCDiagnosticValue[0] & 0x04);					// bitmask for comp_low flag
	COF = (AGCDiagnosticValue[0] & 0x02);						// bitmask for COF flag
	OCF = (AGCDiagnosticValue[0] & 0x01);						// bitmask for OCF flag

	// Set/reset LED if we have/don't have an error
	if(comp_high == 1 || comp_low == 1 || COF == 1 || OCF == 0)
	{
		EnableAlarmLED();
		JCUState.Errors |= ERROR_ENCODER_MAGNET;
		//ClearErrorFlags();
	}
	else
	{
		DisableAlarmLED();
		JCUState.Errors &=~ ERROR_ENCODER_MAGNET;
	}
}

//--------------Calculate PARITY EVEN bit-----------------
bool ParityOk(uint16_t scancode)
{

	uint16_t parity = 0;
	// checking 15 bits for EVEN Parity
	for (uint8_t i = 0; i < 15; i++)
	{
		if(scancode & 0x01)										// see if LSB is 1
		{
			parity++;											// if it 1 add counter of "ones"
		}
		scancode = scancode >> 1;								// shift to next bit
	}

	parity = parity & 0x01;
	// Compare calculated parity and the MSB of received DATA, return "1" if they are the same
	return (parity == scancode);
}
//-------------------------------------------------------

void EncoderDataParcing(void)
{
	switch(EncoderState)
	{
		case STATE_ENCODER_READ_ANGLE:
			JCUState.Angle = (Angle[0] << 8) + Angle[1];
			if (ParityOk(JCUState.Angle))
				JCUState.Errors &=~ ERROR_ENCODER_PARITY;
			else
				JCUState.Errors |= ERROR_ENCODER_PARITY;
			JCUState.Angle &= 0x3FFF;
			break;

		case STATE_ENCODER_READ_ERRORS:
			CheckErrorsEnc();
			break;
		case STATE_ENCODER_CLEAR_ERRORS:

			break;
	}
}


void ReadAngle_RequestErrors(void)
{
	EncoderState = STATE_ENCODER_READ_ANGLE;
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive_IT(&hspi1, AGCDiagnosticAddress, Angle, SIZE);
	//HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
													// remove 2 MSB bits from angle, because they represent Parity and Error
}

void ReadErrors_RequestAngle(void)
{
	EncoderState = STATE_ENCODER_READ_ERRORS;
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive_IT(&hspi1, ReadAngle, AGCDiagnosticValue, SIZE);		// read data about error, and sent request to read angle, so that next communication we would have actual angle information
	//HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);


}

void ClearErrorFlags(void)
{
	EncoderState = STATE_ENCODER_CLEAR_ERRORS;
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive_IT(&hspi1, ReadErrorFlagsAddress, ReadAngle, SIZE);

}

void EnableAlarmLED(void)
{
	HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
}

void DisableAlarmLED(void)
{
	HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
}



/*
 * Legacy code
 */
/*

void ReadAngle_RequestErrors(void)
{
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, AGCDiagnosticAddress, Angle, SIZE, 10);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
	JCUState.Angle = (Angle[0] << 8) + Angle[1];
	if (ParityOk(JCUState.Angle))
		JCUState.Errors &=~ ERROR_ENCODER_PARITY;
	else
		JCUState.Errors |= ERROR_ENCODER_PARITY;
	JCUState.Angle &= 0x3FFF;														// remove 2 MSB bits from angle, because they represent Parity and Error
}

void ReadErrors_RequestAngle(void)
{
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, ReadAngle, AGCDiagnosticValue, SIZE, 10);		// read data about error, and sent request to read angle, so that next communication we would have actual angle information
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
	CheckErrorsEnc();
}

void ClearErrorFlags(void)
{
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, ReadErrorFlagsAddress, 2, 10);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}


void RequestErrors()
{
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, AGCDiagnosticAddress, 2, 10);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}

void ReadErrors()
{

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, ReadAngle, AGCDiagnosticValue, 2, 10);			// read data about error, and sent request to read angle, so that next communication we would have actual angle information
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
	CheckErrorsEnc();
}

void GetAngle(void)
{
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, ReadAngle, Angle, 2, 10);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
	AngularPosition = (Angle[0] << 8) + Angle[1];
	if (ParityOk(AngularPosition))
		EncoderErrorFlag = 0;
	else
		EncoderErrorFlag = 1;
	AngularPosition &= 0x3FFF;														// remove 2 MSB bits from angle, because they represent Parity and Error

}
*/
