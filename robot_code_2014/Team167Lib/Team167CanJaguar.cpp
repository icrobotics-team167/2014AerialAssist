#include "_Team167Lib.h"

/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2009. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/* Modded my Nick Lorch for Team167											  */
/*----------------------------------------------------------------------------*/

#include "CANJaguar.h"
#define tNIRIO_i32 int
#include "ChipObject/NiFpga.h"
#include "CAN/JaguarCANDriver.h"
#include "CAN/can_proto.h"
#include "WPIErrors.h"
#include <stdio.h>

#define swap16(x) ( (((x)>>8) &0x00FF) \
                  | (((x)<<8) &0xFF00) )
#define swap32(x) ( (((x)>>24)&0x000000FF) \
                  | (((x)>>8) &0x0000FF00) \
                  | (((x)<<8) &0x00FF0000) \
                  | (((x)<<24)&0xFF000000) )

#define kFullMessageIDMask (CAN_MSGID_API_M | CAN_MSGID_MFR_M | CAN_MSGID_DTYPE_M)




/**
 * Common initialization code called by all constructors.
 */
/*
void Team167CanJaguar::InitJaguar()
{
	m_transactionSemaphore = semMCreate(SEM_Q_PRIORITY | SEM_INVERSION_SAFE | SEM_DELETE_SAFE);
	if (m_deviceNumber < 1 || m_deviceNumber > 63)
	{
		// Error
		return;
	}
	UINT32 fwVer = GetFirmwareVersion();
	printf("fwVersion[%d]: %d\n", m_deviceNumber, fwVer);
	if (fwVer >= 3330 || fwVer < 86)
	{
		//wpi_assertCleanStatus(kRIOStatusVersionMismatch);
		return;
	}
	switch (m_controlMode)
	{
	case kPercentVbus:
		sendMessage(LM_API_VOLT_T_EN | m_deviceNumber, NULL, 0);
		break;
	default:
		return;
	}
}
*/

/**
 * Constructor
 * 
 * @param deviceNumber The the address of the Jaguar on the CAN bus.
 */
Team167CanJaguar::Team167CanJaguar(UINT8 deviceNumber, ControlMode controlMode): 
	CANJaguar(deviceNumber,controlMode)
{
	rev = 1.0f;

		
	
//	InitJaguar();
}

//Team167CanJaguar::~Team167CanJaguar()
//{
//	semDelete(m_transactionSemaphore);
//	m_transactionSemaphore = NULL;
//}


void Team167CanJaguar::SetForward()
{
	rev = 1.0f;
	
	return;
}

void Team167CanJaguar::SetReverse()
{
	rev = -1.0f;
	
	return;
}

/**
 * Set the output set-point value.  
 * 
 * In PercentVoltage Mode, the input is in the range -1.0 to 1.0
 * 
 * @param outputValue The set-point to sent to the motor controller.
 */
void Team167CanJaguar::Set(float outputValue, UINT8 syncGroup)
{
	outputValue *= rev;
	
	UINT32 messageID;
	UINT8 dataBuffer[8];
	UINT8 dataSize;

	if (m_safetyHelper && !m_safetyHelper->IsAlive())
	{
		EnableControl();
	}

	switch(m_controlMode)
	{
	case kPercentVbus:
		{
			messageID = LM_API_VOLT_T_SET;
			if (outputValue > 1.0) outputValue = 1.0;
			if (outputValue < -1.0) outputValue = -1.0;
			dataSize = packPercentage(dataBuffer, outputValue);
		}
		break;
	case kSpeed:
		{
			messageID = LM_API_SPD_T_SET;
			dataSize = packFXP16_16(dataBuffer, outputValue);
		}
		break;
	case kPosition:
		{
			messageID = LM_API_POS_T_SET;
			dataSize = packFXP16_16(dataBuffer, outputValue);
		}
		break;
	case kCurrent:
		{
			messageID = LM_API_ICTRL_T_SET;
			dataSize = packFXP8_8(dataBuffer, outputValue);
		}
		break;
	case kVoltage:
		{
			messageID = LM_API_VCOMP_T_SET;
			dataSize = packFXP8_8(dataBuffer, outputValue);
		}
		break;
	default:
		return;
	}
	if (syncGroup != 0)
	{
		dataBuffer[dataSize] = syncGroup;
		dataSize++;
	}
	setTransaction(messageID, dataBuffer, dataSize);
	if (m_safetyHelper) m_safetyHelper->Feed();
	
/*	UINT32 messageID;
	UINT8 dataBuffer[8];
	UINT8 dataSize;

	switch(m_controlMode)
	{
	case kPercentVbus:
		{
			messageID = LM_API_VOLT_T_SET;
			dataSize = packPercentage(dataBuffer, outputValue);
		}
		break;
	case kSpeed:
		{
			messageID = LM_API_SPD_T_SET;
			dataSize = packFXP16_16(dataBuffer, outputValue);
		}
		break;
	case kPosition:
		{
			messageID = LM_API_POS_T_SET;
			dataSize = packFXP16_16(dataBuffer, outputValue);
		}
		break;
	case kCurrent:
		{
			messageID = LM_API_ICTRL_T_SET;
			dataSize = packFXP8_8(dataBuffer, outputValue);
		}
		break;
	default:
		return;
	}
	setTransaction(messageID, dataBuffer, dataSize);
	*/
}


/**
 * Get the voltage being output from the motor terminals of the Jaguar.
 * 
 * @return The output voltage in Volts.
 */
/*float Team167CanJaguar::GetOutputVoltage()
{
	UINT8 dataBuffer[8];
	UINT8 dataSize;
	float busVoltage;

	// Read the bus voltage first so we can return units of volts
	busVoltage = GetBusVoltage();
	// Then read the volt out which is in percentage of bus voltage units.
	getTransaction(LM_API_STATUS_VOLTOUT, dataBuffer, &dataSize);
	if (dataSize == sizeof(INT16))
	{
		return busVoltage * unpackPercentage(dataBuffer);
	}
	return 0.0;
}
*/
/**
 * Get the position of the encoder or potentiometer.
 * 
 * @return The position of the motor based on the configured feedback.
 */
double Team167CanJaguar::GetPosition()
{
	UINT8 dataBuffer[8];
	UINT8 dataSize;

	getTransaction(LM_API_STATUS_POS, dataBuffer, &dataSize);
	if (dataSize == sizeof(INT32))
	{
		double new_retval=0;
		double retval = unpackFXP16_16(dataBuffer);
		if(retval > 32768.0)
			new_retval = retval - 65535;
		else
			new_retval =  retval;
		
		return new_retval * (double)rev;
	}
	return 0.0;
}

/**
 * Get the speed of the encoder.
 * 
 * @return The speed of the motor in RPM based on the configured feedback.
 */
/*double Team167CanJaguar::GetSpeed()
{
	UINT8 dataBuffer[8];
	UINT8 dataSize;

	getTransaction(LM_API_STATUS_SPD, dataBuffer, &dataSize);
	if (dataSize == sizeof(INT32))
	{
		return unpackFXP16_16(dataBuffer);
	}
	return 0.0;
}
*/
