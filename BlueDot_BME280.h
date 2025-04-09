#ifndef BLUEDOT_BME280_H
#define BLUEDOT_BME280_H

#define BLUEDOT_BME280_VERSION "1.10.0"

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

// remobe libs if not used
//#define BME280_useI2C
//#define BME280_useHWSPI
#define BME280_useSWI2C

#ifdef BME280_useI2C
#include <Wire.h>
#endif

#ifdef BME280_useHWSPI
#include <SPI.h>
#endif

#ifdef BME280_useSWI2C
//#include "SoftWire.h"
#include "Swire.h"
#endif

#define BME280_CHIP_ID			0xD0
#define BME280_CTRL_HUM			0xF2
#define BME280_Status			0xF3
#define BME280_CTRL_MEAS		0xF4		
#define BME280_CONFIG			0xF5
#define BME280_PRESSURE_MSB		0xF7
#define BME280_PRESSURE_LSB		0xF8
#define BME280_PRESSURE_XLSB	0xF9
#define BME280_TEMPERATURE_MSB	0xFA
#define BME280_TEMPERATURE_LSB	0xFB
#define BME280_TEMPERATURE_XLSB	0xFC
#define BME280_HUMIDITY_MSB		0xFD
#define BME280_HUMIDITY_LSB		0xFE

// set frequecy for SW SPI 
void setSWCom_freq_kHz(uint8_t);

enum Coefficients
{
	BME280_DIG_T1_LSB	=		0x88,
	BME280_DIG_T1_MSB	=		0x89,
	BME280_DIG_T2_LSB	=		0x8A,
	BME280_DIG_T2_MSB	=		0x8B,
	BME280_DIG_T3_LSB	=		0x8C,
	BME280_DIG_T3_MSB	=		0x8D,

	BME280_DIG_P1_LSB	=		0x8E,
	BME280_DIG_P1_MSB	=		0x8F,
	BME280_DIG_P2_LSB	=		0x90,
	BME280_DIG_P2_MSB	=		0x91,
	BME280_DIG_P3_LSB	=		0x92,
	BME280_DIG_P3_MSB	=		0x93,	
	BME280_DIG_P4_LSB	=		0x94,
	BME280_DIG_P4_MSB	=		0x95,
	BME280_DIG_P5_LSB	=		0x96,
	BME280_DIG_P5_MSB	=		0x97,	
	BME280_DIG_P6_LSB	=		0x98,
	BME280_DIG_P6_MSB	=		0x99,	
	BME280_DIG_P7_LSB	=		0x9A,
	BME280_DIG_P7_MSB	=		0x9B,
	BME280_DIG_P8_LSB	=		0x9C,
	BME280_DIG_P8_MSB	=		0x9D,
	BME280_DIG_P9_LSB	=		0x9E,
	BME280_DIG_P9_MSB	=		0x9F,
	
	BME280_DIG_H1		=		0xA1,
	BME280_DIG_H2_LSB	=		0xE1,
	BME280_DIG_H2_MSB	=		0xE2,
	BME280_DIG_H3		=		0xE3,
	BME280_DIG_H4_MSB	=		0xE4,
	BME280_DIG_H4_LSB	=		0xE5,
	BME280_DIG_H5_MSB	=		0xE6,
	BME280_DIG_H6		=		0xE7,
};


struct BME280_Coefficients
{
      uint16_t dig_T1;
      int16_t  dig_T2;
      int16_t  dig_T3;

      uint16_t dig_P1;
      int16_t  dig_P2;
      int16_t  dig_P3;
      int16_t  dig_P4;
      int16_t  dig_P5;
      int16_t  dig_P6;
      int16_t  dig_P7;
      int16_t  dig_P8;
      int16_t  dig_P9;

      uint8_t  dig_H1;
      int16_t  dig_H2;
      uint8_t  dig_H3;
      int16_t  dig_H4;
      int16_t  dig_H5;
      int8_t   dig_H6;
};
	
	
struct DeviceParameter
{
	uint8_t communication;				// 0=I2C, 1=software SPI, 2=Hardware SPI, 3=softwareI2C
	int8_t SPI_cs;
	int8_t SPI_sck;						// SPI_sck also used for SW I2C sck
	int8_t SPI_mosi;					// SPI_mosi also used for SW I2C sda
	int8_t SPI_miso;
	uint8_t I2CAddress;
	uint8_t sensorMode;					// 0=sleep mode, 1=2=forced mode, 3=normal mode
	uint8_t t_sb;						// inactive duration in normal mode
	uint8_t IIRfilter;					// IIR Filter level,
	uint8_t spi3;						// nse 3-wire SPI
	uint8_t tempOversampling;			// ovesampling factor
	uint8_t pressOversampling;			// ovesampling factor
	uint8_t humidOversampling;			// ovesampling factor
	uint16_t pressureSeaLevel;			// default value
	int16_t tempOutsideCelsius;			// default value
	int16_t tempOutsideFahrenheit;		// default value
};


class BlueDot_BME280 
{
 public: 
  BlueDot_BME280();

  DeviceParameter parameter;
  BME280_Coefficients bme280_coefficients;
    
  uint8_t init(void);
  uint8_t checkID(void);

  float readPressure(void);
  float readTempC(void);
  float readTempF(void);
  float readHumidity(void);
  float readAltitudeFeet(void);
  float readAltitudeMeter(void);
  void  readMeasurement(float *valueP, float *valueT, float *valueH);

private:
  int32_t t_fine;		//int temperature needed for presure and huminity calculation

  uint8_t readByte(byte reg);
  uint8_t spiTransfer(uint8_t data);

  float calcHumidity(int32_t raw_H);
  float calcTempC(int32_t row_T);
  float calcTempF(int32_t row_T);
  float calcPressure(int32_t row_P);
  float convertTempKelvin(void);

  void writeByte(byte reg, byte value);
  void writeConfig(void);
  void readCoefficients(void);
  void writeCTRLMeas(void);
};

#endif // BLUEDOT_BME280_H