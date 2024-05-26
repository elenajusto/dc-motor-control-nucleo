/*
 * BMP085-Cube Library
 * Source: https://github.com/tvlad1234/BMP085-Cube/tree/main
 *
 *
 */

#include "bmp085.h"

// Calibration data registers
#define BMP085_CAL_AC1 0xAA //!< R   Calibration data (16 bits)
#define BMP085_CAL_AC2 0xAC //!< R   Calibration data (16 bits)
#define BMP085_CAL_AC3 0xAE //!< R   Calibration data (16 bits)
#define BMP085_CAL_AC4 0xB0 //!< R   Calibration data (16 bits)
#define BMP085_CAL_AC5 0xB2 //!< R   Calibration data (16 bits)
#define BMP085_CAL_AC6 0xB4 //!< R   Calibration data (16 bits)
#define BMP085_CAL_B1 0xB6  //!< R   Calibration data (16 bits)
#define BMP085_CAL_B2 0xB8  //!< R   Calibration data (16 bits)
#define BMP085_CAL_MB 0xBA  //!< R   Calibration data (16 bits)
#define BMP085_CAL_MC 0xBC  //!< R   Calibration data (16 bits)
#define BMP085_CAL_MD 0xBE  //!< R   Calibration data (16 bits)

// Commands
#define BMP085_CONTROL 0xF4         //!< Control register
#define BMP085_TEMPDATA 0xF6        //!< Temperature data register
#define BMP085_PRESSUREDATA 0xF6    //!< Pressure data register
#define BMP085_READTEMPCMD 0x2E     //!< Read temperature control register value
#define BMP085_READPRESSURECMD 0x34 //!< Read pressure control register value


static const uint8_t BMP085_ADDR = 0x77 << 1; // Use 8-bit address
I2C_HandleTypeDef *bmpPort;
uint8_t oversampling;

// Calibration data (will be read from sensor)
int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
uint16_t ac4, ac5, ac6;

// I2C handling functions
uint8_t bmpRead8(uint8_t a)
{
  uint8_t r;
  HAL_I2C_Master_Transmit(bmpPort, BMP085_ADDR, &a, 1, HAL_MAX_DELAY);
  HAL_I2C_Master_Receive(bmpPort, BMP085_ADDR, &r, 1, HAL_MAX_DELAY);
  return r;
}

uint16_t bmpRead16(uint8_t a)
{
  uint8_t retbuf[2];
  uint16_t r;
  HAL_I2C_Master_Transmit(bmpPort, BMP085_ADDR, &a, 1, HAL_MAX_DELAY);
  HAL_I2C_Master_Receive(bmpPort, BMP085_ADDR, retbuf, 2, HAL_MAX_DELAY);
  r = retbuf[1] | (retbuf[0] << 8);
  return r;
}

void bmpWrite8(uint8_t a, uint8_t d)
{
  uint8_t tBuf[2];
  tBuf[0] = a;
  tBuf[1] = d;
  HAL_I2C_Master_Transmit(bmpPort, BMP085_ADDR, tBuf, 2, HAL_MAX_DELAY);
}

uint8_t bmpBegin(uint8_t mode, I2C_HandleTypeDef *i2cdev)
{

  bmpPort = i2cdev;

  if (mode > BMP085_ULTRAHIGHRES)
    mode = BMP085_ULTRAHIGHRES;
  oversampling = mode;

  if (bmpRead8(0xD0) != 0x55)
    return 0;

  /* read calibration data */
  ac1 = bmpRead16(BMP085_CAL_AC1);
  ac2 = bmpRead16(BMP085_CAL_AC2);
  ac3 = bmpRead16(BMP085_CAL_AC3);
  ac4 = bmpRead16(BMP085_CAL_AC4);
  ac5 = bmpRead16(BMP085_CAL_AC5);
  ac6 = bmpRead16(BMP085_CAL_AC6);

  b1 = bmpRead16(BMP085_CAL_B1);
  b2 = bmpRead16(BMP085_CAL_B2);

  mb = bmpRead16(BMP085_CAL_MB);
  mc = bmpRead16(BMP085_CAL_MC);
  md = bmpRead16(BMP085_CAL_MD);

  return 1;
}

// Sensor read functions
int32_t computeB5(int32_t UT)
{
  int32_t X1 = (UT - (int32_t)ac6) * ((int32_t)ac5) >> 15;
  int32_t X2 = ((int32_t)mc << 11) / (X1 + (int32_t)md);
  return X1 + X2;
}

uint16_t readBMPRawTemperature(void)
{
  bmpWrite8(BMP085_CONTROL, BMP085_READTEMPCMD);
  HAL_Delay(5);
  return bmpRead16(BMP085_TEMPDATA);
}

uint32_t readBMPRawPressure(void)
{
  uint32_t raw;

  bmpWrite8(BMP085_CONTROL, BMP085_READPRESSURECMD + (oversampling << 6));

  if (oversampling == BMP085_ULTRALOWPOWER)
    HAL_Delay(5);
  else if (oversampling == BMP085_STANDARD)
    HAL_Delay(8);
  else if (oversampling == BMP085_HIGHRES)
    HAL_Delay(14);
  else
    HAL_Delay(26);

  raw = bmpRead16(BMP085_PRESSUREDATA);

  raw <<= 8;
  raw |= bmpRead8(BMP085_PRESSUREDATA + 2);
  raw >>= (8 - oversampling);

  return raw;
}

float readBMPTemperature(void)
{
  int32_t UT, B5; // following ds convention
  float temp;

  UT = readBMPRawTemperature();

  B5 = computeB5(UT);
  temp = (B5 + 8) >> 4;
  temp /= 10;

  return temp;
}

int32_t readBMPPressure(void)
{
  int32_t UT, UP, B3, B5, B6, X1, X2, X3, p;
  uint32_t B4, B7;

  UT = readBMPRawTemperature();
  UP = readBMPRawPressure();

  B5 = computeB5(UT);

  // do pressure calcs
  B6 = B5 - 4000;
  X1 = ((int32_t)b2 * ((B6 * B6) >> 12)) >> 11;
  X2 = ((int32_t)ac2 * B6) >> 11;
  X3 = X1 + X2;
  B3 = ((((int32_t)ac1 * 4 + X3) << oversampling) + 2) / 4;

  X1 = ((int32_t)ac3 * B6) >> 13;
  X2 = ((int32_t)b1 * ((B6 * B6) >> 12)) >> 16;
  X3 = ((X1 + X2) + 2) >> 2;
  B4 = ((uint32_t)ac4 * (uint32_t)(X3 + 32768)) >> 15;
  B7 = ((uint32_t)UP - B3) * (uint32_t)(50000UL >> oversampling);

  if (B7 < 0x80000000)
  {
    p = (B7 * 2) / B4;
  }
  else
  {
    p = (B7 / B4) * 2;
  }
  X1 = (p >> 8) * (p >> 8);
  X1 = (X1 * 3038) >> 16;
  X2 = (-7357 * p) >> 16;

  p = p + ((X1 + X2 + (int32_t)3791) >> 4);

  return p;
}
