/*
 * AS5048A.h
 *
 *  Created on: Dec 13, 2020
 *      Author: VIKI
 */

#ifndef INC_AS5048A_H_
#define INC_AS5048A_H_

#include "main.h"
#include "JCU_defines.h"
#include <stdbool.h>
#include <stdint.h>

extern SPI_HandleTypeDef hspi1;
extern void DisableMotor(void);

#define SIZE 								2										// number of bytes of the message of SPI communication with AD5048A
#define STATE_ENCODER_READ_ANGLE			1
#define STATE_ENCODER_READ_ERRORS			2
#define STATE_ENCODER_CLEAR_ERRORS			3

/*
 * DEFINES JCU ERRORS Register
 */

#define ERROR_ENCODER_MAGNET_Pos			(0U)									// Encoder magnetic performance error, data received form encoder cannot be trusted
#define ERROR_ENCODER_MAGNET_Msk			(0x1UL << ERROR_ENCODER_MAGNET_Pos)		/*!< 0x00000001 */
#define ERROR_ENCODER_MAGNET				ERROR_ENCODER_MAGNET_Msk

#define ERROR_ENCODER_PARITY_Pos			(1U)									// Encoder parity error, data received form encoder cannot be trusted
#define ERROR_ENCODER_PARITY_Msk			(0x1UL << ERROR_ENCODER_PARITY_Pos)		/*!< 0x00000002 */
#define ERROR_ENCODER_PARITY				ERROR_ENCODER_PARITY_Msk

typedef enum {
	ENCODER_CLEAR_ERRORS,
	ENCODER_LOCK_ERRORS,
	ENCODER_WAIT_ANGLE,
	ENCODER_LOCK_ANGLE,
	ENCODER_BUSY_ANGLE,
	ENCODER_WAIT_DIAGNOSTIC,
	ENCODER_LOCK_DIAGNOSTIC,
	ENCODER_BUSY_DIAGNOSTIC,
} eEncoderStateMachine;

eEncoderStateMachine EncoderState;

/*
 * Functions prototypes
 */

void CheckErrorsEnc(void);										// Function to check errors of the AS5048A
bool ParityOk(uint16_t scancode);								// Check Parity Bit, return TRUE if calculated parity and sent parity are equal
void GetAngle(void);
void ClearErrorFlags(void);										// Clear
void EnableAlarmLED(void);
void DisableAlarmLED(void);
void RequestErrors();
void ReadErrors();
void ReadAngle_RequestErrors(void);
void ReadErrors_RequestAngle(void);
void EncoderDataParcing(void);
void EncoderRoutine(void);

#endif /* INC_AS5048A_H_ */

