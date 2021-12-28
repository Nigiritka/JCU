/*
 * AS5048A.h
 *
 *  Created on: Dec 13, 2020
 *      Author: VIKI
 */

#ifndef INC_AS5048A_H_
#define INC_AS5048A_H_

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

extern SPI_HandleTypeDef hspi1;

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
	ENCODER_CLEAR_ERRORS = 0U,
	ENCODER_LOCK_ERRORS = 1U,
	ENCODER_WAIT_ANGLE = 2U,
	ENCODER_LOCK_ANGLE = 3U,
	ENCODER_BUSY_ANGLE = 4U,
	ENCODER_WAIT_DIAGNOSTIC = 5U,
	ENCODER_LOCK_DIAGNOSTIC = 6U,
	ENCODER_BUSY_DIAGNOSTIC = 7U,
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

