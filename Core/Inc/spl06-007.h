/*
 * spl06-007.h
 *
 *	SPL06-007 Air Pressure Sensor I2C Driver
 *
 *  Created on: May 26, 2024
 *      Author: elena
 */

#ifndef INC_SPL06_007_H_
#define INC_SPL06_007_H_

#include "stm32g0xx_hal.h"				/* For I2C */

/*
 * DEFINES
 */
#define SPL06007_I2C_ADDR		(0x76 << 1)	/* SDO -> GND (p.9) */
#define SPL06007_I2C_ID_ADDR	0x0D		/* Product and Revision ID address (p.27) */
#define SPL06007_I2C_REV_ID		0x10		/* Product and Revision ID reset value (p.27) */

/*
 * REGISTERS (p.17)
 */
#define SPL06_REG_


/*
 * SENSOR STRUCT
 */
typedef struct {
	/* I2C Handle */
	I2C_HandleTypeDef *i2cHandle;

	/* Pressure Data */
	float pressure;

} SPL06_007;

/*
 * INITIALISATION
 */
uint8_t SPL06_007_Initialise( SPL06_007 *dev, I2C_HandleTypeDef *i2cHandle );

/*
 * DATA ACQUISITION
 */
HAL_StatusTypeDef SPL06_007_ReadPressure( SPL06_007 *dev );

/*
 * LOW-LEVEL FUNCTIONS
 */
HAL_StatusTypeDef SPL06_007_ReadRegister( SPL06_007 *dev, uint8_t reg, uint8_t *data );
HAL_StatusTypeDef SPL06_007_WriteRegister( SPL06_007 *dev, uint8_t reg, uint8_t *data );

#endif /* INC_SPL06_007_H_ */
