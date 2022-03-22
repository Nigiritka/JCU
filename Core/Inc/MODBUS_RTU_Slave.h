/*
 * MODBUS_RTU_Slave.h
 *
 *  Created on: 03.07.2021
 *      Author: VIKI
 */

#ifndef INC_MODBUS_RTU_SLAVE_H_
#define INC_MODBUS_RTU_SLAVE_H_


#include "main.h"
#include "Motor_Driver.h"

/*
 * external
 */
extern uint16_t *pJCUConfig;
extern uint16_t *pJCUState;
extern uint8_t RxData[];
extern uint8_t TxData[];
extern UART_HandleTypeDef huart2;
extern CRC_HandleTypeDef hcrc;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;

/*
 * Slave's registers addresses
 *
 * Analog Output Holding Registers
 */

#define ADDRESS_ACCELERATION				0x9C41				// Address to set Acceleration (higher byte) and Deceleration (lower byte) for the JCU
#define ADDRESS_SPEED_TORQUE				0x9C42				// Address to set desired max speed (higher byte) and desired max torque (lower byte)
#define ADDRESS_TARGET_ANGLE				0x9C43				// Address to set Target position (target angle)
#define ADDRESS_STATUS_REGISTER				0x9C44

#define ADDRESS_KP_COEF_CURR_PID_HI			0x9C45				// Address of proportional coefficient of PID current loop, high register
#define ADDRESS_KP_COEF_CURR_PID_LO			0x9C46				// Address of proportional coefficient of PID current loop, low register
#define ADDRESS_KI_COEF_CURR_PID_HI			0x9C47				// Address of integral coefficient of PID current loop, high register
#define ADDRESS_KI_COEF_CURR_PID_LO			0x9C48				// Address of integral coefficient of PID current loop, low register
#define ADDRESS_KD_COEF_CURR_PID_HI			0x9C49				// Address of derivative coefficient of PID current loop, high register
#define ADDRESS_KD_COEF_CURR_PID_LO			0x9C4A				// Address of derivative coefficient of PID current loop, low register

#define ADDRESS_KP_COEF_POS_PID_HI			0x9C4B				// Address of proportional coefficient of PID position loop, high register
#define ADDRESS_KP_COEF_POS_PID_LO			0x9C4C				// Address of proportional coefficient of PID position loop, low register
#define ADDRESS_KI_COEF_POS_PID_HI			0x9C4D				// Address of integral coefficient of PID position loop, high register
#define ADDRESS_KI_COEF_POS_PID_LO			0x9C4E				// Address of integral coefficient of PID position loop, low register
#define ADDRESS_KD_COEF_POS_PID_HI			0x9C4F				// Address of derivative coefficient of PID position loop, high register
#define ADDRESS_KD_COEF_POS_PID_LO			0x9C50				// Address of derivative coefficient of PID position loop, low register

#define ADDRESS_KP_COEF_SPEED_PID_HI		0x9C51				// Address of proportional coefficient of PID speed loop, high register
#define ADDRESS_KP_COEF_SPEED_PID_LO		0x9C52				// Address of proportional coefficient of PID speed loop, low register
#define ADDRESS_KI_COEF_SPEED_PID_HI		0x9C53				// Address of integral coefficient of PID speed loop, high register
#define ADDRESS_KI_COEF_SPEED_PID_LO		0x9C54				// Address of integral coefficient of PID speed loop, low register
#define ADDRESS_KD_COEF_SPEED_PID_HI		0x9C55				// Address of derivative coefficient of PID speed loop, high register
#define ADDRESS_KD_COEF_SPEED_PID_LO		0x9C56				// Address of derivative coefficient of PID speed loop, low register


#define TOTAL_AOHR							22					// Total amount of Analog Output Holding Registers available in the Slave

#define TOTAL_COILS							16					// Total amount of Coils (bits in Status Register)

#define WRITE_MULTIPLE_AOHR_BYTES_RESPONSE	8					// Fixed size for response when multiple registers were written




/*
 * Analog Input Registers
 */

#define ADDRESS_JCU_ERRORS					0x7531
#define ADDRESS_JCU_ANGLE					0x7532
#define ADDRESS_JCU_SPEED					0x7533
#define ADDRESS_JCU_TORQUE					0x7534
#define ADDRESS_JCU_TEMP					0x7535

#define TOTAL_AIR							5					// Total amount of Analog Input Registers available in the Slave

/*
 * Modbus function codes
 */
#define READ_COILS							0x01				// Read coils status
#define READ_AOHR							0x03				// Read Analog Output Holding Registers
#define READ_AIR							0x04				// Read Analog Input Registers
#define WRITE_SINGLE_COIL					0x05				// Write single coil
#define WRITE_SINGLE_AOHR					0x06				// Write single Analog Output Holding Registers
#define WRITE_MULTIPLE_AOHR					0x10				// Write multiple Analog Output Holding Registers

/*
 * Modbus Exception codes
 */

#define MODBUS_ILLEGAL_FUNCTION 			0x01				// The function code received in the query is not an allowable action for the slave.
#define MODBUS_ILLEGAL_DATA_ADDRESS			0x02				// The data address received in the query is not an allowable address for the slave.

const uint16_t polynom = 0xA001;								// polynom for CRC calculation
const uint8_t SLAVE_ID = 1;										// unique slave address
const uint8_t SLAVE_ID_BROADCAST = 0;							// SLAVE ID for Broadcast requests from MASTER


void ModbusRTURoutine(uint8_t *pBUFFER, uint8_t Length);
void ModbusExceptionHandler(uint8_t ExceptionCode);




#endif /* INC_MODBUS_RTU_SLAVE_H_ */
