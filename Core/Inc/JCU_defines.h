
/*
 * DEFINES CONTROL Register
 */

#define ENABLE_MOTOR_Pos					(1U)									// Enable timer which control PWM for the motor
#define ENABLE_MOTOR_Msk					(0x1UL << ENABLE_MOTOR_Pos)				/*!< 0x00000010 */
#define ENABLE_MOTOR						ENABLE_MOTOR_Msk

#define DISABLE_MOTOR_Pos					(2U)									// Disable Motor (Disable timer which control PWM for motor)
#define DISABLE_MOTOR_Msk					(0x1UL << SET_BRAKE_Pos)				/*!< 0x00000100 */
#define DISABLE_MOTOR						DISABLE_MOTOR_Msk

#define SET_BRAKE_Pos						(3U)									// Enable Brake
#define SET_BRAKE_Msk						(0x1UL << SET_BRAKE_Pos)				/*!< 0x00001000 */
#define SET_BRAKE							SET_BRAKE_Msk

#define GO_TO_TARGET_POSITION_Pos			(4U)									// Motor will go to target position which is set in JCU status structure
#define GO_TO_TARGET_POSITION_Msk			(0x1UL << GO_TO_TARGET_POSITION_Pos)	/*!< 0x00010000 */
#define GO_TO_TARGET_POSITION				GO_TO_TARGET_POSITION_Msk

#define STOP_MOTOR_Pos						(5U)									// Stop motor in current position, PWM still will be apply 50%
#define STOP_MOTOR_Msk						(0x1UL << STOP_MOTOR_Pos)				/*!< 0x00100000 */
#define STOP_MOTOR							STOP_MOTOR_Msk

#define SOFTWARE_RESET_Pos					(15U)									// Software reset. initiate software reset when is set to 1
#define SOFTWARE_RESET_Msk					(0x1UL << SOFTWARE_RESET_Pos)				/*!< 0x100000000 00000000 */
#define SOFTWARE_RESET						SOFTWARE_RESET_Msk

/*
 * Defines Error register
 */

#define REACHED_TARGET_POSITION_Pos			(5U)									// FLAG if motor reached target position
#define REACHED_TARGET_POSITION_Msk			(0x1UL << REACHED_TARGET_POSITION_Pos)	/*!< 0x00010000 */
#define REACHED_TARGET_POSITION				REACHED_TARGET_POSITION_Msk

#define BRAKE_STATUS_Pos					(3U)									// this bit is set if brake is set
#define BRAKE_STATUS_Msk					(0x1UL << BRAKE_STATUS_Pos)				/*!< 0x00000100 */
#define BRAKE_STATUS						BRAKE_STATUS_Msk

typedef struct
{
	uint8_t Acceleration;
	uint8_t Deceleration;
	uint8_t MaxSpeed;
	uint8_t MaxTorque;
	uint16_t TargetAngel;
	uint16_t ControlRegister;
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





