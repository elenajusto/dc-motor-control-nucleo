/*
 * spl06-007.c
 *
 *	SPL06-007 Air Pressure Sensor I2C Driver
 *
 *  Created on: May 26, 2024
 *      Author: elena
 */

/* INCLUDES */
#include "spl06-007.h"

/*
 * INITIALISATION
 */
uint8_t SPL06_007_Initialise( SPL06_007 *dev, I2C_HandleTypeDef *i2cHandle ){

	/* Set struct parameters */
	dev->i2cHandle = i2cHandle;

	dev->compensatedPressure = 0.0f;

	dev->compensatedTemperature = 0.0f;

	dev->scaleFactor = 2088960;

	/* Store number of transaction errors (to be returned at end of function */
	uint8_t errNum = 0;
	HAL_StatusTypeDef status;

	/* Check device Product and Revision ID (DATASHEET PAGE 27) */
	uint8_t regData;
	status = SPL06_007_ReadRegister( dev, SPL06007_I2C_ID_ADDR, &regData);
	errNum += ( status != HAL_OK );	/* Increment error count if error countered */

	if ( regData != SPL06007_I2C_REV_ID){
		return 255;
	}

	/* Set Sensor Operating Mode and Status (MEAS_CFG) */
	/* Hard coded to: 111 - Continuous pressure and temperature measurement */
	uint8_t setRegValueMEAS = 0xC7;
	status = SPL06_007_WriteRegister(dev, SPL06_REG_MEAS_CFG_ADDR, &setRegValueMEAS);
	errNum += ( status != HAL_OK );	/* Increment error count if error countered */

	/* Set Pressure Configuration (PRS_CFG) measurement rate and over sampling rate */
	/* Hard coded to: PM_RATE[2:0] = 111 - 128 measurements pr. sec.
					  PM_PRC[3:0] = 0110 *) - 64 times (High Precision) */
	uint8_t setRegValuePRS = 0x77;
	status = SPL06_007_WriteRegister(dev, SPL06_REG_PRS_CFG_ADDR, &setRegValueMEAS);
	errNum += ( status != HAL_OK );	/* Increment error count if error countered */

	/* Set Temperature Configuration (TMP_CFG) measurement rate and over sampling rate */
	/* Hard coded to: TMP_RATE[2:0] = 111 - 128 measurements pr. sec.
					  TMP_PRC[2:0] = 111 - 128 times. */
	uint8_t setRegValueTMP = 0xF7;
	status = SPL06_007_WriteRegister(dev, SPL06_REG_TMP_CFG_ADDR, &setRegValueMEAS);
	errNum += ( status != HAL_OK );	/* Increment error count if error countered */


	/* Set Interrupt and FIFO configuration (CFG_REG) */
	/* Hard coded to: T_SHIFT = Must be set to '1' when the oversampling rate is >8 times.
					  P_SHIFT = Must be set to '1' when the oversampling rate is >8 times. */
	uint8_t setRegValueCFG = 0xC;
	status = SPL06_007_WriteRegister(dev, SPL06_REG_CFG_REG_ADDR, &setRegValueMEAS);
	errNum += ( status != HAL_OK );	/* Increment error count if error countered */

	/* Return number of errors */
	return errNum;					/* 0 means successful setup */
}

/*
 * DATA ACQUISITION
 */
uint8_t SPL06_007_checkMode( SPL06_007 *dev ){

	/* Access Sensor Operating Mode and Status (MEAS_CFG) Register (0x08) */
	uint8_t regData;
	SPL06_007_ReadRegister( dev, SPL06_REG_MEAS_CFG_ADDR, &regData);

	/* Return value */
	return regData;
}

uint8_t SPL06_007_calcCompPressure( SPL06_007 *dev ){

}

uint8_t SPL06_007_calcCompTemp( SPL06_007 *dev ){

}

uint8_t SPL06_007_getRawPressure( SPL06_007 *dev ){

}

uint8_t SPL06_007_getRawTemp( SPL06_007 *dev ){

}

/*
 * LOW-LEVEL FUNCTIONS
 */
HAL_StatusTypeDef SPL06_007_ReadRegister( SPL06_007 *dev, uint8_t reg, uint8_t *data ){

	return HAL_I2C_Mem_Read(dev->i2cHandle, SPL06007_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);

}

HAL_StatusTypeDef SPL06_007_WriteRegister( SPL06_007 *dev, uint8_t reg, uint8_t *data ){

	return HAL_I2C_Mem_Write(dev->i2cHandle, SPL06007_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

/* FUNCTION DEFINITIONS */




