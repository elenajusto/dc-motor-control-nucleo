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

	dev->pressure = 0.0f;

	/* Store number of transaction errors (to be returned at end of function */
	uint8_t errNum = 0;
	HAL_StatusTypeDef status;

	/* Check device Product and Revision ID (DATASHEET PAGE 27) */
	uint8_t regData;

	status = SPL06_007_ReadRegister( dev, SPL06007_I2C_ID_ADDR, &regData);
	errNum += ( status != HAL_OK );

	if ( regData != SPL06007_I2C_REV_ID){
		return 255;
	}

	/* Configure sensor settings*/

	/* Return number of errors */
	return errNum;					// 0 means successful setup
}

/*
 * DATA ACQUISITION
 */
uint8_t SPL06_007_checkMode( SPL06_007 *dev ){

	/* Access Sensor Operating Mode and Status (MEAS_CFG) Register (0x08) */
	HAL_StatusTypeDef status;
	uint8_t regData;
	status = SPL06_007_ReadRegister( dev, SPL06_REG_MEAS_CFG_ADDR, &regData);

	/* Return value */
	return regData;
}

HAL_StatusTypeDef SPL06_007_ReadPressure( SPL06_007 *dev );

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




