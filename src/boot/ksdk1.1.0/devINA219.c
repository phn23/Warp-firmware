#include <stdlib.h>

/*
 *	config.h needs to come first
 */
#include "config.h"

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"
// added
#include "devINA219.h"



extern volatile WarpI2CDeviceState	deviceINA219State;
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;



uint16_t INA219_currentMultiplier_mA; // = 0;
// float ina219_powerMultiplier_mW;  // = 0.0f;
uint16_t INA219_calValue;



void
initINA219(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts)
{
	deviceINA219State.i2cAddress			= i2cAddress;
	deviceINA219State.operatingVoltageMillivolts	= operatingVoltageMillivolts;
	
	// Activate calibration and reg
	setCalibration_INA219(); // TODO: UPDATE NAME
	// setCalibration_16V_400mA();

	return;
}



// WRITE
// All INA219 registers 16-bit registers are actually two 8-bit bytes via the I2C interface -> 2 bytes
WarpStatus
writeSensorRegisterINA219(uint8_t deviceRegister, uint16_t payload)
{
	uint8_t		payloadByte[2], commandByte[1]; // 2 data bytes
	i2c_status_t	status;

	switch (deviceRegister)
	{
		// there are 6 registers
		// can only write on 2
		case 0x00:  
		case 0x05:
		{
			/* OK */
			break;
		}

		default:
		{
			return kWarpStatusBadDeviceCommand;
		}
	}

	i2c_device_t slave =
		{
		.address = deviceINA219State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);

	commandByte[0] = deviceRegister;
	payloadByte[0] = (payload >> 8) & 0xFF; /* MSB first */
	payloadByte[1] = payload & 0xFF;        /* LSB */
	warpEnableI2Cpins();

/*!
 * @brief Performs a blocking receive transaction on the I2C bus.
 *
 * Both cmdBuff and rxBuff are byte aligned, user needs to prepare these buffers
 * according to related protocol if slave devices data are not byte-aligned. 
 *
 * @param instance    The I2C peripheral instance number.
 * @param device      The pointer to the I2C device information structure.
 * @param cmdBuff     The pointer to the commands to be transferred, could be NULL.
 * @param cmdSize     The length in bytes of the commands to be transferred, could be 0.
 * @param rxBuff      The pointer to the data to be transferred, cannot be NULL.
 * @param rxSize      The length in bytes of the data to be transferred, cannot be 0.
 * @param timeout_ms  A timeout for the transfer in microseconds.
 * @return Error or success status returned by API.
 */	
	status = I2C_DRV_MasterSendDataBlocking(
		0 /* I2C instance - not sure about this value*/,
		&slave,
		commandByte,
		1,
		payloadByte,
		2 /* 2 data bytes needed */,
		gWarpI2cTimeoutMilliseconds);
	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}


// CONFIGURE
// not sure why this function is needed here
// in the MMA8451Qexample the input  0x00, 0x01
// my guess is that it enables the sensors to reset
// disable FIFO with 0x00
// activate ctrl_reg1 with 0x01
// TODO: Determine whether this is needed
// WarpStatus
// configureSensorINA219(uint8_t payloadConfig_Reg, uint8_t payload_Calibration)
// {
// 	WarpStatus	i2cWriteStatus1, i2cWriteStatus2;


// 	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);

// 	i2cWriteStatus1 = writeSensorRegisterINA219(kWarpSensorConfigurationRegisterINA219F_SETUP /* register address F_SETUP */,
// 												  payloadF_SETUP /* payload: Disable FIFO */
// 	);

// 	i2cWriteStatus2 = writeSensorRegisterINA219(kWarpSensorConfigurationRegisterINA219CTRL_REG1 /* register address CTRL_REG1 */,
// 												  payloadCTRL_REG1 /* payload */
// 	);

// 	return (i2cWriteStatus1 | i2cWriteStatus2); // error bit-wise operation
// }


// READ
WarpStatus
readSensorRegisterINA219(uint8_t deviceRegister, int numberOfBytes)
{
	uint8_t		cmdBuf[1] = {0xFF};
	i2c_status_t	status;


	USED(numberOfBytes);
	switch (deviceRegister)
	{
		// there are 6 registers
		// can read all 6
		case INA219_REG_CONFIG: 
		case INA219_REG_SHUNTVOLTAGE: 
		case INA219_REG_BUSVOLTAGE: 
		case INA219_REG_POWER:
		case INA219_REG_CURRENT:
		case INA219_REG_CALIBRATION:
		{
			/* OK */
			break;
		}

		default:
		{
			return kWarpStatusBadDeviceCommand;
		}
	}

	i2c_device_t slave =
		{
		.address = deviceINA219State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
	cmdBuf[0] = deviceRegister;
	warpEnableI2Cpins();

	status = I2C_DRV_MasterReceiveDataBlocking(
		0 /* I2C peripheral instance */,
		&slave,
		cmdBuf,
		1,
		(uint8_t *)deviceINA219State.i2cBuffer,
		numberOfBytes,
		gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

// PRINT
void
printSensorDataINA219(bool hexModeFlag)
{
	uint16_t	readSensorRegisterValueLSB;
	uint16_t	readSensorRegisterValueMSB;
	int16_t		readSensorRegisterValueCombined;
	WarpStatus	i2cReadStatus;


	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);

	/*
	 *	From the MMA8451Q datasheet:
	 *
	 *		"A random read access to the LSB registers is not possible.
	 *		Reading the MSB register and then the LSB register in sequence
	 *		ensures that both bytes (LSB and MSB) belong to the same data
	 *		sample, even if a new data sample arrives between reading the
	 *		MSB and the LSB byte."
	 *
	 *	We therefore do 2-byte read transactions, for each of the registers.
	 *	We could also improve things by doing a 6-byte read transaction.
	 */

	// shunt, bus, current, power
	i2cReadStatus = readSensorRegisterINA219(INA219_REG_SHUNTVOLTAGE, 2 /* numberOfBytes */); 
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF); // CHANGE THESE

	/*
	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
	 */
	
	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB); // HEX
		}
		else
		{
			warpPrint(" %d,", readSensorRegisterValueCombined); // DECIMAL
		}
	}


	i2cReadStatus = readSensorRegisterINA219(INA219_REG_BUSVOLTAGE, 2 /* numberOfBytes */); 
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

	/*
	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
	 */

	
	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			warpPrint(" %d,", readSensorRegisterValueCombined);
		}
	}


	i2cReadStatus = readSensorRegisterINA219(INA219_REG_CURRENT, 2 /* numberOfBytes */); 
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

	/*
	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
	 */
	
	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			warpPrint(" %d,", readSensorRegisterValueCombined);
		}
	}



	i2cReadStatus = readSensorRegisterINA219(INA219_REG_POWER, 2 /* numberOfBytes */); 
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

	/*
	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
	 */
	
	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			warpPrint(" %d,", readSensorRegisterValueCombined);
		}
	} 	
}

// APPEND
uint8_t
appendSensorDataINA219(uint8_t* buf)
{
	uint8_t index = 0;
	uint16_t readSensorRegisterValueLSB;
	uint16_t readSensorRegisterValueMSB;
	int16_t readSensorRegisterValueCombined;
	WarpStatus i2cReadStatus;

	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);

	/*
	 *	From the MMA8451Q datasheet:
	 *
	 *		"A random read access to the LSB registers is not possible.
	 *		Reading the MSB register and then the LSB register in sequence
	 *		ensures that both bytes (LSB and MSB) belong to the same data
	 *		sample, even if a new data sample arrives between reading the
	 *		MSB and the LSB byte."
	 *
	 *	We therefore do 2-byte read transactions, for each of the registers.
	 *	We could also improve things by doing a 6-byte read transaction.
	 */
	i2cReadStatus                   = readSensorRegisterINA219(INA219_REG_SHUNTVOLTAGE, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB      = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB      = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

	/*
	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
	 */


	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

	if (i2cReadStatus != kWarpStatusOK)
	{
		buf[index] = 0;
		index += 1;

		buf[index] = 0;
		index += 1;
	}
	else
	{
		/*
		 * MSB first
		 */
		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 8);
		index += 1;

		buf[index] = (uint8_t)(readSensorRegisterValueCombined);
		index += 1;
	}

	i2cReadStatus                   = readSensorRegisterINA219(INA219_REG_BUSVOLTAGE, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB      = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB      = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

	/*
	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
	 */


	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

	if (i2cReadStatus != kWarpStatusOK)
	{
		buf[index] = 0;
		index += 1;

		buf[index] = 0;
		index += 1;
	}
	else
	{
		/*
		 * MSB first
		 */
		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 8);
		index += 1;

		buf[index] = (uint8_t)(readSensorRegisterValueCombined);
		index += 1;
	}

	i2cReadStatus                   = readSensorRegisterINA219(INA219_REG_CURRENT, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB      = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB      = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

	/*
	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
	 */

	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

	if (i2cReadStatus != kWarpStatusOK)
	{
		buf[index] = 0;
		index += 1;

		buf[index] = 0;
		index += 1;
	}
	else
	{
		/*
		 * MSB first
		 */
		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 8);
		index += 1;

		buf[index] = (uint8_t)(readSensorRegisterValueCombined);
		index += 1;
	}
	
	i2cReadStatus                   = readSensorRegisterINA219(INA219_REG_POWER, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB      = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB      = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);
	/*
	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
	 */

	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

	if (i2cReadStatus != kWarpStatusOK)
	{
		buf[index] = 0;
		index += 1;

		buf[index] = 0;
		index += 1;
	}
	else
	{
		/*
		 * MSB first
		 */
		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 8);
		index += 1;

		buf[index] = (uint8_t)(readSensorRegisterValueCombined);
		index += 1;
	}
	
	return index;
}

/*******************************************************************
// ADDED FOR CALIBRATION
*******************************************************************/
// int16_t INA219_getBusVoltage_raw() {
//   uint16_t value;

// readSensorRegisterINA219(INA219_REG_BUSVOLTAGE, 2);


//   Adafruit_BusIO_Register bus_voltage_reg =
//       Adafruit_BusIO_Register(i2c_dev, INA219_REG_BUSVOLTAGE, 2, MSBFIRST);
//   _success = bus_voltage_reg.read(&value);

//   // Shift to the right 3 to drop CNVR and OVF and multiply by LSB
//   return (int16_t)((value >> 3) * 4);
// }

// /*!
//  *  @brief  Gets the raw shunt voltage (16-bit signed integer, so +-32767)
//  *  @return the raw shunt voltage reading
//  */
// int16_t INA219_getShuntVoltage_raw() {
//   uint16_t value;
//   Adafruit_BusIO_Register shunt_voltage_reg =
//       Adafruit_BusIO_Register(i2c_dev, INA219_REG_SHUNTVOLTAGE, 2, MSBFIRST);
//   _success = shunt_voltage_reg.read(&value);
//   return value;
// }

// // /*!
// //  *  @brief  Gets the raw power value (16-bit signed integer, so +-32767)
// //  *  @return raw power reading
// //  */
// // int16_t INA219_getPower_raw() {
// //   uint16_t value;

// //   // Sometimes a sharp load will reset the INA219, which will
// //   // reset the cal register, meaning CURRENT and POWER will
// //   // not be available ... avoid this by always setting a cal
// //   // value even if it's an unfortunate extra step
// //   Adafruit_BusIO_Register calibration_reg =
// //       Adafruit_BusIO_Register(i2c_dev, INA219_REG_CALIBRATION, 2, MSBFIRST);
// //   calibration_reg.write(ina219_calValue, 2);

// //   // Now we can safely read the POWER register!
// //   Adafruit_BusIO_Register power_reg =
// //       Adafruit_BusIO_Register(i2c_dev, INA219_REG_POWER, 2, MSBFIRST);
// //   _success = power_reg.read(&value);
// //   return value;
// // }

// /*!
//  *  @brief  Gets the shunt voltage in mV (so +-327mV)
//  *  @return the shunt voltage converted to millivolts
//  */
// float INA219_getShuntVoltage_mV() {
//   int16_t value;
//   value = getShuntVoltage_raw();
//   return value * 0.01;
// }

// /*!
//  *  @brief  Gets the bus voltage in volts
//  *  @return the bus voltage converted to volts
//  */
// float INA219_getBusVoltage_V() {
//   int16_t value = getBusVoltage_raw();
//   return value * 0.001;
// }



// /*!
//  *  @brief  Gets the power value in mW, taking into account the
//  *          config settings and current LSB
//  *  @return power reading converted to milliwatts
//  */
// float INA219_getPower_mW() {
//   float valueDec = getPower_raw();
//   valueDec *= ina219_powerMultiplier_mW;
//   return valueDec;
// }

  

/*!
 *  @brief set device to alibration which uses the highest precision for
 *     current measurement (0.1mA), at the expense of
 *     only supporting 16V at 400mA max.
 */


/*!
 *  @brief  Configures to INA219 to be able to measure up to 16V and 0.4A
 *          of current.  Each unit of current corresponds to 100uA, and
 *          each unit of power corresponds to 2mW. Counter overflow
 *          occurs at 3.2A.
 *  @note   These calculations assume a 0.1 ohm resistor is present
 */


/************************************************************************
WE ONLY NEED CURRENT
*************************************************************************/
/*!
 *  @brief  Gets the raw current value (16-bit signed integer, so +-32767)
 *  @return the raw current reading
 */
int32_t INA219_getCurrent_mA() {
	uint16_t  readSensorRegisterValueMSB;
	uint16_t  readSensorRegisterValueLSB;
	int16_t  readSensorRegisterValueCombined; // -/+ 32767
	int32_t valueDec;	

	// Sometimes a sharp load will reset the INA219, which will
	// reset the cal register, meaning CURRENT and POWER will
	// not be available ... avoid this by always setting a cal
	// value even if it's an unfortunate extra step

	// first calibrate
	writeSensorRegisterINA219(INA219_REG_CALIBRATION, INA219_calValue);
	
	// Now we can safely read the CURRENT register!
	// now we read
	

	readSensorRegisterINA219(INA219_REG_CURRENT, 2 /* numberOfBytes */); // read 2 bytes from current reg
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = (((readSensorRegisterValueMSB & 0xFF) << 8) | readSensorRegisterValueLSB); // USE THIS BECAUSE  NO NEED 

	// multiply by LSB unit
	valueDec = readSensorRegisterValueCombined *  0.1; // INA219_currentMultiplier_mA; // TODO
	
	return valueDec;
}



/************************************************************************
END - WE ONLY NEED CURRENT
*************************************************************************/


/**********************************************************
callibration test
**********************************************************/


void setCalibration_INA219(){

  // Calibration which uses the highest precision for
  // current measurement (0.1mA), at the expense of
  // only supporting 16V at 400mA max.

  // VBUS_MAX = 16V
  // VSHUNT_MAX = 0.04          (Assumes Gain 1, 40mV)
  // RSHUNT = 0.1               (Resistor value in ohms)

  // 1. Determine max possible current
  // MaxPossible_I = VSHUNT_MAX / RSHUNT
 // MaxPossible_I = 0.4 (A)

  // 2. Determine max expected current
  // MaxExpected_I = 0.075A (75 mA)

  // 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
  // MinimumLSB = MaxExpected_I/32767
  // MinimumLSB = 0.0000023             (2.3uA per bit)
  // MaximumLSB = MaxExpected_I/4096
  // MaximumLSB = 0.0000183              (18uA per bit)

  // 4. Choose an LSB between the min and max values
  //    (Preferrably a roundish number close to MinLSB)
  // CurrentLSB = 0.000010 (10 uA per bit) // LSB is 6.3 uA/bit

  // 5. Compute the calibration register
  // Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
  // Cal = 40960 

/**********************************************************************
	calibrated value
***********************************************************************/
  uint16_t ina219_calValue = 40960; // SMALLER THAN 65535


/*****************************************************************************
	got lost here
*****************************************************************************/
	

  // 6. Calculate the power LSB
  // PowerLSB = 20 * CurrentLSB
  // PowerLSB = 0.001 (1mW per bit)

  // 7. Compute the maximum current and shunt voltage values before overflow
  //
  // Max_Current = Current_LSB * 32767
  // Max_Current = 0.32767A before overflow
  //
  // If Max_Current > Max_Possible_I then (0.32767 < 0.4)
  //    Max_Current_Before_Overflow = MaxPossible_I
  // Else
  //    Max_Current_Before_Overflow = Max_Current
  // End If
//	
	// Max_Current_Before_Overflow = 0.32767 A
  
	
  // Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
  // Max_ShuntVoltage = 0.03277V
  //
  // If Max_ShuntVoltage >= VSHUNT_MAX
  //    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
  // Else
  //    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
  // End If

  // Max_ShuntVoltage_Before_Overflow = 0.03277V

  // 8. Compute the Maximum Power
  // MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
  // MaximumPower = 0.32767 * 0.03277V
  // MaximumPower = 0.01W

  // Set multipliers to convert raw current/power values
  // ina219_currentDivider_mA = 20;    // Current LSB = 10uA per bit (1000/10 = 100)	
  INA219_currentMultiplier_mA = 0.01 ;    // Current LSB = 10uA per bit = 0.01 mA per bit 	
  // ina219_powerMultiplier_mW = 1.0f; // Power LSB = 1mW per bit

  // Set Calibration register to 'Cal' calculated above
  writeSensorRegisterINA219(INA219_REG_CALIBRATION, 8192) ; // TODO: ina219_calValue=91);
	  

  // Set Config register to take into account the settings above
  uint16_t config = INA219_CONFIG_BVOLTAGERANGE_16V |
                    INA219_CONFIG_GAIN_1_40MV | INA219_CONFIG_BADCRES_12BIT |
                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

writeSensorRegisterINA219(INA219_REG_CONFIG, config);
}

// plan get the calibrated current
// multiply by LSB (unit)





/********************************************************************
EXAMPLE FUNCTION
*************************************************************************/


void setCalibration_16V_400mA() {

  // Calibration which uses the highest precision for
  // current measurement (0.1mA), at the expense of
  // only supporting 16V at 400mA max.

  // VBUS_MAX = 16V
  // VSHUNT_MAX = 0.04          (Assumes Gain 1, 40mV)
  // RSHUNT = 0.1               (Resistor value in ohms)

  // 1. Determine max possible current
  // MaxPossible_I = VSHUNT_MAX / RSHUNT
  // MaxPossible_I = 0.4A

  // 2. Determine max expected current
  // MaxExpected_I = 0.4A

  // 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
  // MinimumLSB = MaxExpected_I/32767
  // MinimumLSB = 0.0000122              (12uA per bit)
  // MaximumLSB = MaxExpected_I/4096
  // MaximumLSB = 0.0000977              (98uA per bit)

  // 4. Choose an LSB between the min and max values
  //    (Preferrably a roundish number close to MinLSB)
  // CurrentLSB = 0.00005 (50uA per bit)

  // 5. Compute the calibration register
  // Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
  // Cal = 8192 (0x2000)

  INA219_calValue = 8192;

  // 6. Calculate the power LSB
  // PowerLSB = 20 * CurrentLSB
  // PowerLSB = 0.001 (1mW per bit)

  // 7. Compute the maximum current and shunt voltage values before overflow
  //
  // Max_Current = Current_LSB * 32767
  // Max_Current = 1.63835A before overflow
  //
  // If Max_Current > Max_Possible_I then
  //    Max_Current_Before_Overflow = MaxPossible_I
  // Else
  //    Max_Current_Before_Overflow = Max_Current
  // End If
  //
  // Max_Current_Before_Overflow = MaxPossible_I
  // Max_Current_Before_Overflow = 0.4
  //
  // Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
  // Max_ShuntVoltage = 0.04V
  //
  // If Max_ShuntVoltage >= VSHUNT_MAX
  //    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
  // Else
  //    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
  // End If
  //
  // Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
  // Max_ShuntVoltage_Before_Overflow = 0.04V

  // 8. Compute the Maximum Power
  // MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
  // MaximumPower = 0.4 * 16V
  // MaximumPower = 6.4W

  // Set multipliers to convert raw current/power values
  INA219_currentMultiplier_mA = 20;    // Current LSB = 50uA per bit (1000/50 = 20)
  // ina219_powerMultiplier_mW = 1.0f; // Power LSB = 1mW per bit

  // Set Calibration register to 'Cal' calculated above




  // Set Calibration register to 'Cal' calculated above
  writeSensorRegisterINA219(INA219_REG_CALIBRATION, INA219_calValue);
	  

  // Set Config register to take into account the settings above
  uint16_t config = INA219_CONFIG_BVOLTAGERANGE_16V |
                    INA219_CONFIG_GAIN_1_40MV | INA219_CONFIG_BADCRES_12BIT |
                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

writeSensorRegisterINA219(INA219_REG_CONFIG, config);
}
