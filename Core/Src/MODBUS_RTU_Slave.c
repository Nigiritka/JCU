/*
 * MODBUS_RTU_Slave.c
 *
 *  Created on: 03.07.2021
 *      Author: VIKI
 */


#include "MODBUS_RTU_Slave.h"


/* 40 us recieving data
 * 18 us waiting silence
 * 2.2 us to start
 * 2.3 us for loop
 * 4.7 us CRC
 * 1.4 us data processing
 * 5.5 us start sending the answer
 * 43.5 us data is sent, 8 bytes
 */


void ModbusRTURoutine(uint8_t *pBUFFER, uint8_t Length)
{
	//Length = BufferSize - hdma_usart1_rx.Instance->CNDTR;
	/*
	 * create temporary variables for ModBus parcing
	 */
	uint16_t CRCCalc = 0;							// CRC which will be calculated
	uint16_t CRCValue = 0;							// CRC which was sent by Master
	uint8_t ModbusFunction = 0;						// Master modbus function (what action slave should do)
	uint16_t RequestedAddress = 0;					// started address which Master wants to read
	uint16_t AmountofRead = 0;						// amount of registers which master wants to read
	uint16_t AmountofWrite = 0;						// amount of registers which master wants to write
	uint8_t ByteCount = 0;							// amount of BYTEs which will be sent in response
	uint16_t CRCforResponse	= 0;					// CRC will be calculated for response data
	uint8_t temp[Length];							// temporary array which we are gonna use to process incoming data
	uint16_t *ptemp16bitJCUConfig = pJCUConfig;		// temporary pointer to shift between data which should be read or written pJCUConfig
	uint16_t *ptemp16bitJCUState = pJCUState;		// temporary pointer to shift between data which should be read or written pJCUState

	// putting all data to the temp array
	for (uint8_t i = 0; i < Length; i++)
	{
		temp [i] = pBUFFER[i];
	}

	// comparing received and calculated CRCs

	CRCValue = temp[Length-1];
	CRCValue <<= 8;
	CRCValue = CRCValue + temp[Length-2];
	CRCCalc = HAL_CRC_Calculate(&hcrc, (uint32_t *)&temp, (Length-2));

	// If data was not corrupted (CRC is ok)
	//if (1)
	if (CRCCalc == CRCValue)
	{
		// Check if the message for us
		if (temp[0] == SLAVE_ID)
		{
			// this packet for us, let's read it
			// Defining the modbus function
			ModbusFunction = temp[1];

			switch (ModbusFunction)
			{
				case READ_COILS:
					// this part is reading JCUState.StatusRegister bit by bit. Detailed description of each bit(coils) in Motor_Driver.h

					// Defining the first address of coil Master wants to read (third and forth bytes of the message):
					RequestedAddress = (temp[2] << 8) + temp[3];

					// Defining how many coil master wants to read
					AmountofRead = (temp[4] << 8) + temp[5];
					ByteCount = AmountofRead*2;

					/*
					 * Check if there is no such errors:
					 * 1. First address of the coil is correct
					 * 2. Amount of the information which master wants to read does not exceed amount of data available in the slave
					 */
					if (((RequestedAddress >= ENABLE_MOTOR_Pos) && (RequestedAddress <= SOFTWARE_RESET_Pos))
							&& (((RequestedAddress - ENABLE_MOTOR_Pos) + AmountofRead) <= TOTAL_COILS))
					{
						TxData[0] = SLAVE_ID;
						TxData[1] = ModbusFunction;
						//TxData[2] = ByteCount;
						uint16_t tempvalue = JCUConfig.StatusRegister;
						if (RequestedAddress < 8)
						{

							tempvalue <<= 8 + (8 - RequestedAddress); 	// move one byte, and other bits from byte which we do not need
							tempvalue >>= 8 + (8 - RequestedAddress);	// fill emptiness with zeros on the left
							TxData[4] = tempvalue;
							tempvalue >>= 8;
							TxData[3] = tempvalue;
						}
						else
						{
							TxData[4] = tempvalue;
							tempvalue >>= 8;
							TxData[3] = tempvalue;
						}
						CRCforResponse = HAL_CRC_Calculate(&hcrc, (uint32_t *)&TxData, (ByteCount+3));
						TxData[ByteCount+3] = CRCforResponse;
						CRCforResponse >>= 8;
						TxData[ByteCount+4] = CRCforResponse;
						while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_BUSY))
						{

						}
						HAL_GPIO_WritePin(RS485_FC_GPIO_Port, RS485_FC_Pin, GPIO_PIN_SET);
						HAL_UART_Transmit_DMA(&huart1, TxData, (5+ByteCount));

					}
					else
					{
						// Modbus Exception 0x02, The data address received in the query is not an allowable address for the slave.
						ModbusExceptionHandler(MODBUS_ILLEGAL_DATA_ADDRESS);
					}



					break;

				case READ_AOHR:
					// Read JCU parameters which Master has set (Acceleration/Deceleration/Max Speed/Max Torque/Target Angle/Status Register)

					// Defining the first address of Holding registers Master wants to read (third and forth bytes of the message):
					RequestedAddress = (temp[2] << 8) + temp[3];
					// Offset the pointer from the first Holding register to desired data.
					ptemp16bitJCUConfig += RequestedAddress - ADDRESS_ACCELERATION;

					// Defining how many Holding registers master wants to read
					AmountofRead = (temp[4] << 8) + temp[5];
					ByteCount = AmountofRead*2;

					/*
					 * Check if there is no such errors:
					 * 1. First address of the Holding Register is correct
					 * 2. Amount of the information which master wants to read does not exceed amount of data available in the slave
					 */
					if (((RequestedAddress >= ADDRESS_ACCELERATION) && (RequestedAddress <= ADDRESS_KD_COEF_SPEED_PID_LO))
							&& (((RequestedAddress - ADDRESS_ACCELERATION) + AmountofRead) <= TOTAL_AOHR))
					{
						TxData[0] = SLAVE_ID;
						TxData[1] = ModbusFunction;
						TxData[2] = ByteCount;

						for (uint8_t i=0; i<ByteCount; i++)
						{
							uint16_t tempvalue = 0;
							tempvalue = *ptemp16bitJCUConfig;
							TxData[i*2 + 4] = tempvalue;
							tempvalue >>= 8;
							TxData[i*2 + 3] = tempvalue;
							ptemp16bitJCUConfig++;
						}
						CRCforResponse = HAL_CRC_Calculate(&hcrc, (uint32_t *)&TxData, (ByteCount+3));
						TxData[ByteCount+3] = CRCforResponse;
						CRCforResponse >>= 8;
						TxData[ByteCount+4] = CRCforResponse;
						while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_BUSY))
						{
							// wait until the line is available
						}
						HAL_GPIO_WritePin(RS485_FC_GPIO_Port, RS485_FC_Pin, GPIO_PIN_SET);
						HAL_UART_Transmit_DMA(&huart1, TxData, (5+ByteCount));
					}

					// if there are errors in request:
					else
					{
						// Modbus Exception 0x02, The data address received in the query is not an allowable address for the slave.
						ModbusExceptionHandler(MODBUS_ILLEGAL_DATA_ADDRESS);
					}
					break;

				case READ_AIR:
					// Read JCU parameters (errors/Angle/speed/torque/temperature)

					// Defining the first address of input registers Master wants to read (third and forth bytes of the message):
					RequestedAddress = (temp[2] << 8) + temp[3];
					// Offset the pointer from the first input register to desired data.
					ptemp16bitJCUState += RequestedAddress - ADDRESS_JCU_ERRORS;

					// Defining how many input registers master wants to read
					AmountofRead = (temp[4] << 8) + temp[5];
					ByteCount = AmountofRead*2;

					/*
					 * Check if there is no such errors:
					 * 1. First address of the input Register is correct
					 * 2. Amount of the information which master wants to read does not exceed amount of data available in the slave
					 */
					if (((RequestedAddress >= ADDRESS_JCU_ERRORS) && (RequestedAddress <= ADDRESS_JCU_TEMP))
							&& (((RequestedAddress - ADDRESS_JCU_ERRORS) + AmountofRead) <= TOTAL_AIR))
					{
						TxData[0] = SLAVE_ID;
						TxData[1] = ModbusFunction;
						TxData[2] = ByteCount;

						for (uint8_t i=0; i<AmountofRead; i++)
						{
							uint16_t tempvalue = 0;
							tempvalue = *ptemp16bitJCUState;
							TxData[i*2 + 4] = tempvalue;
							tempvalue >>= 8;
							TxData[i*2 + 3] = tempvalue;
							ptemp16bitJCUState++;
						}
						CRCforResponse = HAL_CRC_Calculate(&hcrc, (uint32_t *)&TxData, (ByteCount+3));
						TxData[ByteCount+3] = CRCforResponse;
						CRCforResponse >>= 8;
						TxData[ByteCount+4] = CRCforResponse;
						while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_BUSY))
						{

						}
						HAL_GPIO_WritePin(RS485_FC_GPIO_Port, RS485_FC_Pin, GPIO_PIN_SET);
						HAL_UART_Transmit_DMA(&huart1, TxData, (5+ByteCount));
					}

					// if there are errors in request:
					else
					{
						// Modbus Exception 0x02, The data address received in the query is not an allowable address for the slave.
						ModbusExceptionHandler(MODBUS_ILLEGAL_DATA_ADDRESS);
					}

					break;

				case WRITE_SINGLE_COIL:

					break;

				case WRITE_SINGLE_AOHR:
					// Write Single parameter to JCU (Acceleration/Deceleration/Max Speed/Max Torque/Target Angle/Status Register)

					RequestedAddress = (temp[2] << 8) + temp[3];
					// Offset the pointer from the first Holding register to desired data.
					ptemp16bitJCUConfig += RequestedAddress - ADDRESS_ACCELERATION;

					/*
					 * Check if there is no such error:
					 * 1. the address of the holding Register is correct
					 */
					if ((RequestedAddress >= ADDRESS_ACCELERATION) && (RequestedAddress <= ADDRESS_KD_COEF_SPEED_PID_LO))
					{
						*ptemp16bitJCUConfig = (temp[4] << 8) + temp[5];

						// send ECHO
						// Maybe better do not sending what we received, but send what we really have in memory as the echo
						HAL_GPIO_WritePin(RS485_FC_GPIO_Port, RS485_FC_Pin, GPIO_PIN_SET);
						while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_BUSY))
						{
							// wait until the line is available
						}
						HAL_UART_Transmit_DMA(&huart1, RxData, Length);
						if (RequestedAddress == ADDRESS_STATUS_REGISTER)
						{
							CheckStatusRegister();
						}

					}
					// if there are errors in request:
					else
					{
						// Modbus Exception 0x02, The data address received in the query is not an allowable address for the slave.
						ModbusExceptionHandler(MODBUS_ILLEGAL_DATA_ADDRESS);
					}
					break;

				case WRITE_MULTIPLE_AOHR:
					// Write multiple parameter to JCU (Acceleration/Deceleration/Max Speed/Max Torque/Target Angle/Status Register)

					// Defining the first address of Holding registers Master wants to write (third and forth bytes of the message):
					RequestedAddress = (temp[2] << 8) + temp[3];
					// Offset the pointer from the first Holding register to desired data.
					ptemp16bitJCUConfig += RequestedAddress - ADDRESS_ACCELERATION;
					// Defining how many Holding registers master wants to read
					AmountofWrite = (temp[4] << 8) + temp[5];
					ByteCount = temp[6];

					/*
					 * Check if there is no such errors:
					 * 1. First address of the Holding Register is correct
					 * 2. Amount of the information which master wants to write does not exceed amount possible infromation
					 * 3. Check if master quantity of Bytes in message is equal (quantity of register) * 2
					 */
					if (((RequestedAddress >= ADDRESS_ACCELERATION) && (RequestedAddress <= ADDRESS_KD_COEF_SPEED_PID_LO))
							&& (((RequestedAddress - ADDRESS_ACCELERATION) + AmountofWrite) <= TOTAL_AOHR)
							&& (ByteCount == AmountofWrite*2))
					{
						//uint8_t *ptemp8bit = (uint8_t*) ptemp16bitJCUState;			// for more convenient shifting of bytes during writing, we are creating 8 bit pointer

						for (uint8_t i=0; i<ByteCount; i+=2)
						{
							*ptemp16bitJCUConfig = (temp[i+7] << 8) + temp[i+8];
							ptemp16bitJCUConfig++;
						}

						// send ECHO
						// Maybe better do not sending what we received, but send what we really have in memory as the echo
						for (uint8_t i = 0; i < 6; i++)
						{
							TxData[i] = temp[i];
						}
						CRCforResponse = HAL_CRC_Calculate(&hcrc, (uint32_t *)&TxData, (ByteCount+2));
						TxData[ByteCount+2] = CRCforResponse;
						CRCforResponse >>= 8;
						TxData[ByteCount+3] = CRCforResponse;
						HAL_GPIO_WritePin(RS485_FC_GPIO_Port, RS485_FC_Pin, GPIO_PIN_SET);
						while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_BUSY))
						{
							// wait until the line is available
						}
						HAL_UART_Transmit_DMA(&huart1, TxData, WRITE_MULTIPLE_AOHR_BYTES_RESPONSE);
						CheckStatusRegister();												// could be improved if we can check, was this register updated by master or not
					}

					// if there are errors in request:
					else
					{
						// Modbus Exception 0x02, The data address received in the query is not an allowable address for the slave.
						ModbusExceptionHandler(MODBUS_ILLEGAL_DATA_ADDRESS);
					}


					break;

				default:
					// Modbus Exception 0x01 - Illegal Function
					ModbusExceptionHandler(MODBUS_ILLEGAL_FUNCTION);
					break;
			}

		}
		else if(temp[0] == SLAVE_ID_BROADCAST)
		{
			//define what to do with broadcast messages
		}
		else
		{
			// do nothing
			// wait new packet
			HAL_GPIO_WritePin(RS485_FC_GPIO_Port, RS485_FC_Pin, GPIO_PIN_RESET);
			HAL_UART_Receive_DMA(&huart1, RxData, 100);
		}
	}
	else
	{
		// do nothing
		// wait new packet
		HAL_GPIO_WritePin(RS485_FC_GPIO_Port, RS485_FC_Pin, GPIO_PIN_RESET);
		HAL_UART_Receive_DMA(&huart1, RxData, 100);
	}



}

void ModbusExceptionHandler(uint8_t ExceptionCode)
{
	// TO DO - implement exception code 01 and 02 for modbus
	if (ExceptionCode == MODBUS_ILLEGAL_FUNCTION)
	{

	}
	else if (ExceptionCode == MODBUS_ILLEGAL_DATA_ADDRESS)
	{

	}
}



