#if defined(_AVR_)
#include <util/delay.h>
#endif

#include "BlueDot_BME280.h"

// delay used for SW SPI
uint32_t SW_SPI_delay_us;

// set frequecy for SW SPI (delay after each SCK change)
void setSWSPI_freq_kHz (uint32_t freq) {
	SW_SPI_delay_us = 500 / freq; 
}


BlueDot_BME280::BlueDot_BME280()
{
	parameter.communication = 0;			// 0=I2C, 1=software SPI or 2=Hardware SPI
	parameter.I2CAddress = 0x77;
	parameter.sensorMode = 0b11;  			// 0=sleep mode, 1=forced mode, 2=normal mode
	// differances BME280 vs. BMP280
	// t_sb		BMP280	BME280
	// 0b110	2s		10ms
	// 0b111	4s		20ms
	parameter.t_sb = 0b101;       			// 0b011 = 250 msec, 0b101 = 1s sampling delay
	parameter.IIRfilter = 0b010;  			// IIR Filter level, 0=filter off, 2=filter 4, 3=filter 8
	parameter.spi3 = 0b0;  					// no 3-wire SPI 
	parameter.tempOversampling = 0b001;		// ovesampling factor, 1=single sampling
	parameter.pressOversampling = 0b001;	// ovesampling factor, 1=single sampling
	parameter.humidOversampling = 0b001;	// ovesampling factor, 1=single sampling
	parameter.pressureSeaLevel = 1013.25;   // default value of 1013.25 hPa
	parameter.tempOutsideCelsius = 15;      // default value of 15°C
	parameter.tempOutsideFahrenheit = 59;   // default value of 59°F

}

uint8_t BlueDot_BME280::init(void)
{
	//In order to start the BME280 we need to go through the following steps:
	//1. Choose the Communication Protocol (I2C, Software SPI or Hardware SPI)
	//2. Read Compensation Coefficients
	//3. Set up IIR Filter (Infinite Impulse Response Filter)
	//4. Set up Oversampling Factors and Sensor Run Mode
	//5. Check Communication (ask and verificate chip ID)	
	
	
	

	//1. Communication Protocol
	//#########################
	//
	
	if (parameter.communication == 1)							//Software SPI Communication		
	{
		pinMode(parameter.SPI_cs, OUTPUT);						//Chip Select Pin as Output
		digitalWrite(parameter.SPI_cs, HIGH);					//Chip Select Pin to HIGH
		pinMode(parameter.SPI_sck, OUTPUT);						//Clock as Output
		pinMode(parameter.SPI_mosi, OUTPUT);					//MOSI as Output
		if (!parameter.spi3)
			pinMode(parameter.SPI_miso, INPUT);					//MISO as Input		
	}
	else if (parameter.communication == 2)						//Hardware SPI Communication
	{
		pinMode(parameter.SPI_cs, OUTPUT);						//Chip Select Pin as Output
		digitalWrite(parameter.SPI_cs, HIGH);					//Chip Select Pin to HIGH
		#ifdef BME280_useHWSPI
		SPI.begin();											//Initialize SPI library
		SPI.setBitOrder(MSBFIRST);								//Most significant Bit first
		SPI.setClockDivider(SPI_CLOCK_DIV4);					//Sets SPI clock to 1/4th of the system clock (i.e. 4000 kHz for Arduino Uno)
		SPI.setDataMode(SPI_MODE0);								//Set Byte Transfer to (0,0) Mode
		#endif
	}
	else														//Default I2C Communication 
	{
		#ifdef BME280_useI2C
		Wire.begin();											//Default value for Arduino Boards
		//Wire.begin(0,2);										//Use this for NodeMCU board; SDA = GPIO0 = D3; SCL = GPIO2 = D4
		#endif
	}


 
		
	//2. Reading Compensation Coefficients
	//####################################
	//After a measurement the device gives values for temperature, pressure and humidity
	//These are all uncompensated and uncablibrated values
	//To correct these values we need the calibration coefficients
	//These are stored into the device during production	
	readCoefficients();
	
	
	
	//3. Set up IIR Filter, normal mode sampling rate (t_sb) and 3-wire spi (3spi)
	//############################################################################
	//The BME280 features an internal IIR (Infinite Impulse Response) Filter
	//The IIR Filter suppresses high frequency fluctuations (i. e. pressure changes due to slamming doors)
	//It improves the pressure and temperature resolution to 20 bits
	//The resolution of the humidity measurement is fixed at 16 bits and is not affected by the filter
	//When enabled, we can set up the filter coefficient (2, 4, 8 or 16)
	//This coefficient defines the filter's time constant (please refer to Datasheet)
	//Parameter t_sb sets the internal sampling rate, independant of the rate you actually poll values.
	// Parameter spi3 turns on 3-wire SPI mode.
	writeConfig();		
	
	
	
	//4. Set up Oversampling Factors and Sensor Mode
	//##############################################
	//Oversampling heavily influences the noise in the data (please refer to the Datasheet for more Info)
	//The BME280 Datasheet provides settings suggestions for different applications		
	//Finally we write all those values to their respective registers
	writeCTRLMeas();
	
	
	
	//5. Check Communication
	//######################
	//All BME280 devices share the same chip ID: 0x60
	//If we read anything else than 0x60, we interrupt the program
	//In this case, please check the wiring to the device
	//Also check the correct I2C Address (either 0x76 or 0x77)
	
	return checkID();

}
//##########################################################################
//SET UP FUNCTIONS
//##########################################################################
uint8_t BlueDot_BME280::checkID(void)
{
	uint8_t chipID;
	chipID = readByte(BME280_CHIP_ID);
	return chipID;
	
}
//##########################################################################
void BlueDot_BME280::readCoefficients(void)
{
	bme280_coefficients.dig_T1 = ((uint16_t)(readByte(BME280_DIG_T1_MSB) << 8) + readByte(BME280_DIG_T1_LSB));
	bme280_coefficients.dig_T2 = ((int16_t)(readByte(BME280_DIG_T2_MSB) << 8) + readByte(BME280_DIG_T2_LSB));
	bme280_coefficients.dig_T3 = ((int16_t)(readByte(BME280_DIG_T3_MSB) << 8) + readByte(BME280_DIG_T3_LSB));
	
	bme280_coefficients.dig_P1 = ((uint16_t)(readByte(BME280_DIG_P1_MSB) << 8) + readByte(BME280_DIG_P1_LSB));
	bme280_coefficients.dig_P2 = ((int16_t)(readByte(BME280_DIG_P2_MSB) << 8) + readByte(BME280_DIG_P2_LSB));
	bme280_coefficients.dig_P3 = ((int16_t)(readByte(BME280_DIG_P3_MSB) << 8) + readByte(BME280_DIG_P3_LSB));
	bme280_coefficients.dig_P4 = ((int16_t)(readByte(BME280_DIG_P4_MSB) << 8) + readByte(BME280_DIG_P4_LSB));
	bme280_coefficients.dig_P5 = ((int16_t)(readByte(BME280_DIG_P5_MSB) << 8) + readByte(BME280_DIG_P5_LSB));
	bme280_coefficients.dig_P6 = ((int16_t)(readByte(BME280_DIG_P6_MSB) << 8) + readByte(BME280_DIG_P6_LSB));
	bme280_coefficients.dig_P7 = ((int16_t)(readByte(BME280_DIG_P7_MSB) << 8) + readByte(BME280_DIG_P7_LSB));
	bme280_coefficients.dig_P8 = ((int16_t)(readByte(BME280_DIG_P8_MSB) << 8) + readByte(BME280_DIG_P8_LSB));
	bme280_coefficients.dig_P9 = ((int16_t)(readByte(BME280_DIG_P9_MSB) << 8) + readByte(BME280_DIG_P9_LSB));
	
	bme280_coefficients.dig_H1 = ((uint8_t)(readByte(BME280_DIG_H1)));
	bme280_coefficients.dig_H2 = ((int16_t)(readByte(BME280_DIG_H2_MSB) << 8) + readByte(BME280_DIG_H2_LSB));
	bme280_coefficients.dig_H3 = ((uint8_t)(readByte(BME280_DIG_H3)));
	bme280_coefficients.dig_H4 = ((int16_t)((readByte(BME280_DIG_H4_MSB) << 4) + (readByte(BME280_DIG_H4_LSB) & 0x0F)));
	bme280_coefficients.dig_H5 = ((int16_t)((readByte(BME280_DIG_H5_MSB) << 4) + ((readByte(BME280_DIG_H4_LSB) >> 4 ) & 0x0F)));
	bme280_coefficients.dig_H6 = ((uint8_t)(readByte(BME280_DIG_H6)));
}
//##########################################################################
void BlueDot_BME280::writeConfig(void)
{
	//The irritatingly vague name (writeConfig) is derived from the data sheet
	//The manufacturers simply refer to byte 0xF5 as "config" so I use it for consistency.
	//It is not really a very good function name.
	//
	//We set up the IIR Filter through bits 4, 3 and 2 from Config Register (0xF5)]
	//t_sb is set in bits 5,6,7 and controls sampling rate
    //spi3 is set in bit 0 and turns on/off the 3pin SPI mode
    //bit 1 is actually never used
    //Please refer to the BME280 Datasheet for more information
	
	byte value;
	value  = (parameter.t_sb << 5) & 0b11100000;
    value |= (parameter.IIRfilter << 2) & 0b00011100;
    value |= parameter.spi3 & 0b00000001;
    writeByte(BME280_CONFIG, value);
}
//##########################################################################
void BlueDot_BME280::writeCTRLMeas(void)
{
	byte value;
	value = parameter.humidOversampling & 0b00000111;
	writeByte(BME280_CTRL_HUM, value);
	
	value = (parameter.tempOversampling << 5) & 0b11100000;
	value |= (parameter.pressOversampling << 2) & 0b00011100;
	value |= parameter.sensorMode & 0b00000011;
	writeByte(BME280_CTRL_MEAS, value);	
}
//##########################################################################
//DATA READOUT FUNCTIONS
//##########################################################################
float BlueDot_BME280::calcPressure(int32_t row_P) {
	int64_t var1, var2, P;
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)bme280_coefficients.dig_P6;
	var2 = var2 + ((var1 * (int64_t)bme280_coefficients.dig_P5)<<17);
	var2 = var2 + (((int64_t)bme280_coefficients.dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)bme280_coefficients.dig_P3)>>8) + ((var1 * (int64_t)bme280_coefficients.dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)bme280_coefficients.dig_P1)>>33;
	if (var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}
	P = 1048576 - row_P;
	P = (((P << 31) - var2)*3125)/var1;
	var1 = (((int64_t)bme280_coefficients.dig_P9) * (P >> 13) * (P >> 13)) >> 25;
	var2 = (((int64_t)bme280_coefficients.dig_P8) * P) >> 19;
	P = ((P + var1 + var2) >> 8) + (((int64_t)bme280_coefficients.dig_P7)<<4);
	
	P = P >> 8; // /256
	return (float)P/100;
}

//##########################################################################
float BlueDot_BME280::readPressure(void)
{
	if (parameter.pressOversampling == 0b000)						//disabling the pressure measurement function
	{
		return 0;
	}
	
	else
	{
		readTempC();
		
		int32_t adc_P;
		adc_P = (uint32_t)readByte(BME280_PRESSURE_MSB) << 12;
		adc_P |= (uint32_t)readByte(BME280_PRESSURE_LSB) << 4;
		adc_P |= (readByte(BME280_PRESSURE_XLSB) >> 4 )& 0b00001111;
		
		return calcPressure(adc_P);
	}
}

//##########################################################################
float BlueDot_BME280::convertTempKelvin(void)
{
	//Temperature in Kelvin is needed for the conversion of pressure to altitude	
	//Both tempOutsideCelsius and tempOutsideFahrenheit are set to 999 as default (see .h file)
	//If the user chooses to read temperature in Celsius, tempOutsideFahrenheit remains 999
	//If the user chooses to read temperature in Fahrenheit instead, tempOutsideCelsius remains 999
	//If both values are used, then the temperature in Celsius will be used for the conversion
	//If none of them are used, then the default value of 288.15 will be used (i.e. 273.15 + 15)
		
	float tempOutsideKelvin;	
	
	if (parameter.tempOutsideCelsius != 999 && parameter.tempOutsideFahrenheit == 999 )   
	{
		tempOutsideKelvin = parameter.tempOutsideCelsius;
		tempOutsideKelvin = tempOutsideKelvin + 273.15;
		return tempOutsideKelvin;		
	}
	
	if (parameter.tempOutsideCelsius != 999 && parameter.tempOutsideFahrenheit != 999 )   
	{
		tempOutsideKelvin = parameter.tempOutsideCelsius;
		tempOutsideKelvin = tempOutsideKelvin + 273.15;
		return tempOutsideKelvin;		
	}
	
	if (parameter.tempOutsideFahrenheit != 999 && parameter.tempOutsideCelsius == 999)
	{
		
		tempOutsideKelvin = (parameter.tempOutsideFahrenheit - 32);
		tempOutsideKelvin = tempOutsideKelvin * 5;
		tempOutsideKelvin = tempOutsideKelvin / 9;
		tempOutsideKelvin = tempOutsideKelvin + 273.15;
		return tempOutsideKelvin;	
	}
	
	if (parameter.tempOutsideFahrenheit == 999 && parameter.tempOutsideCelsius == 999)
	{
		tempOutsideKelvin = 273.15 + 15;
		return tempOutsideKelvin; 
	}
	
	tempOutsideKelvin = 273.15 + 15;
	return tempOutsideKelvin;
}

//##########################################################################
float BlueDot_BME280::readAltitudeFeet(void)
{	
	float heightOutput = 0;
	float tempOutsideKelvin = convertTempKelvin();
	
	heightOutput = readPressure();
	heightOutput = (heightOutput/parameter.pressureSeaLevel);
	heightOutput = pow(heightOutput, 0.190284);
	heightOutput = 1 - heightOutput;
	heightOutput = heightOutput * tempOutsideKelvin;
	heightOutput = heightOutput / 0.0065;
	heightOutput = heightOutput / 0.3048;
	return heightOutput;
}

//##########################################################################
float BlueDot_BME280::readAltitudeMeter(void)
{
	float heightOutput = 0;
	float tempOutsideKelvin = convertTempKelvin();
	
	heightOutput = readPressure();
	heightOutput = (heightOutput/parameter.pressureSeaLevel);
	heightOutput = pow(heightOutput, 0.190284);
	heightOutput = 1 - heightOutput;	
	heightOutput = heightOutput * tempOutsideKelvin;
	heightOutput = heightOutput / 0.0065;
	return heightOutput;	
}

//##########################################################################
float BlueDot_BME280::calcHumidity(int32_t raw_H) {
	int32_t var1;
	var1 = (t_fine - ((int32_t)76800));
	var1 = (((((raw_H << 14) - (((int32_t)bme280_coefficients.dig_H4) << 20) - (((int32_t)bme280_coefficients.dig_H5) * var1)) +
	((int32_t)16384)) >> 15) * (((((((var1 * ((int32_t)bme280_coefficients.dig_H6)) >> 10) * (((var1 * ((int32_t)bme280_coefficients.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
	((int32_t)bme280_coefficients.dig_H2) + 8192) >> 14));
	var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int32_t)bme280_coefficients.dig_H1)) >> 4));
	var1 = (var1 < 0 ? 0 : var1);
	var1 = (var1 > 419430400 ? 419430400 : var1);
	float H = (var1>>12);
	H = H /1024.0;
	return H;
}

//##########################################################################
float BlueDot_BME280::readHumidity(void)
{
	if (parameter.humidOversampling == 0b000)					//disabling the humitidy measurement function
	{
		return 0;
	}
	
	else
	{
		int32_t adc_H;
		adc_H = (uint32_t)readByte(BME280_HUMIDITY_MSB) << 8;
		adc_H |= (uint32_t)readByte(BME280_HUMIDITY_LSB);
		
		return calcHumidity(adc_H);
	}
}

//##########################################################################
float BlueDot_BME280::calcTempC(int32_t row_T) {
	int64_t var1, var2;
	
	var1 = ((((row_T>>3) - ((int32_t)bme280_coefficients.dig_T1<<1))) * ((int32_t)bme280_coefficients.dig_T2)) >> 11;
	var2 = (((((row_T>>4) - ((int32_t)bme280_coefficients.dig_T1)) * ((row_T>>4) - ((int32_t)bme280_coefficients.dig_T1))) >> 12) *
	((int32_t)bme280_coefficients.dig_T3)) >> 14;
	t_fine = var1 + var2;
	float T = (t_fine * 5 + 128) >> 8;
	T = T / 100;
	return T;
}

//##########################################################################
float BlueDot_BME280::readTempC(void)
{
	if (parameter.tempOversampling == 0b000)					//disabling the temperature measurement function
	{
		return 0;
	}
	
	else
	{
		int32_t adc_T;
		adc_T = (uint32_t)readByte(BME280_TEMPERATURE_MSB) << 12;
		adc_T |= (uint32_t)readByte(BME280_TEMPERATURE_LSB) << 4;
		adc_T |= (readByte(BME280_TEMPERATURE_XLSB) >> 4 )& 0b00001111;
		
		return calcTempC(adc_T);
	}
}

//##########################################################################
float BlueDot_BME280::calcTempF(int32_t row_T) {
	int64_t var1, var2;
	
	var1 = ((((row_T>>3) - ((int32_t)bme280_coefficients.dig_T1<<1))) * ((int32_t)bme280_coefficients.dig_T2)) >> 11;
	var2 = (((((row_T>>4) - ((int32_t)bme280_coefficients.dig_T1)) * ((row_T>>4) - ((int32_t)bme280_coefficients.dig_T1))) >> 12) *
	((int32_t)bme280_coefficients.dig_T3)) >> 14;
	t_fine = var1 + var2;
	float T = (t_fine * 5 + 128) >> 8;
	T = T / 100;
	T = (T * 1.8) + 32;
	return T;
}

//##########################################################################
float BlueDot_BME280::readTempF(void)
{
	if (parameter.tempOversampling == 0b000)				//disabling the temperature measurement function
	{
		return 0;
	}	
	
	else
	{
		int32_t adc_T;
		adc_T = (uint32_t)readByte(BME280_TEMPERATURE_MSB) << 12;
		adc_T |= (uint32_t)readByte(BME280_TEMPERATURE_LSB) << 4;
		adc_T |= (readByte(BME280_TEMPERATURE_XLSB) >> 4 )& 0b00001111;
		
		return calcTempF(adc_T);
	}
}

//##########################################################################
//BASIC FUNCTIONS
//##########################################################################
void BlueDot_BME280::writeByte(byte reg, byte value)
{
	if (parameter.communication == 1)					//Software SPI
	{
		digitalWrite(parameter.SPI_cs, LOW);
		spiTransfer(reg & 0x7F);
		spiTransfer(value);
		digitalWrite(parameter.SPI_cs, HIGH);	
	}
	
	else if (parameter.communication == 2)				//Hardware SPI
	{
		digitalWrite(parameter.SPI_cs, LOW);
		#ifdef BME280_useHWSPI
		SPI.transfer(reg & 0x7F);
		SPI.transfer(value);
		#endif
		digitalWrite(parameter.SPI_cs, HIGH);
	}
	
	else												//I2C (default)
	{
		#ifdef BME280_useI2C
		Wire.beginTransmission(parameter.I2CAddress);
		Wire.write(reg);
		Wire.write(value);
		Wire.endTransmission();
		#endif
	}
}
//##########################################################################
uint8_t BlueDot_BME280::readByte(byte reg)
{
	uint8_t value;
	if (parameter.communication == 1)					//Software SPI
	{
		digitalWrite(parameter.SPI_cs, LOW);
		spiTransfer(reg | 0x80);		
		value = spiTransfer(0);
		digitalWrite(parameter.SPI_cs, HIGH);		
		return value;
	}
	
	else if (parameter.communication == 2)				//Hardware SPI
	{
		digitalWrite(parameter.SPI_cs, LOW);
		#ifdef BME280_useHWSPI
		SPI.transfer(reg | 0x80);		
		value = SPI.transfer(0);
		#endif
		digitalWrite(parameter.SPI_cs, HIGH);		
		return value;
	}
	
	else												//I2C (default)
	{
		#ifdef BME280_useI2C
		Wire.beginTransmission(parameter.I2CAddress);
		Wire.write(reg);
		Wire.endTransmission();
		Wire.requestFrom(parameter.I2CAddress,(uint8_t)1);		
		value = Wire.read();
		#endif	
		return value;
	}
}

//##########################################################################
void BlueDot_BME280::readMeasurement(float *valueP, float *valueT, float *valueH)		//burst read of sensor data
{
	int32_t row_P, row_T, row_H;
	if (parameter.communication == 1)					//Software SPI
	{
		digitalWrite(parameter.SPI_cs, LOW);
		spiTransfer(BME280_PRESSURE_MSB | 0x80);		

		row_P = (uint32_t)spiTransfer(0)  << 12;
		row_P |= (uint32_t)spiTransfer(0)  << 4;
		row_P |= (spiTransfer(0)  >> 4 )& 0b00001111;
		row_T = (uint32_t)spiTransfer(0) << 12;
		row_T |= (uint32_t)spiTransfer(0) << 4;
		row_T |= (spiTransfer(0) >> 4 )& 0b00001111;
		row_H = (uint32_t)spiTransfer(0) << 8;
		row_H |= (uint32_t)spiTransfer(0);
		digitalWrite(parameter.SPI_cs, HIGH);	
	}
	
	else if (parameter.communication == 2)				//Hardware SPI
	{
		digitalWrite(parameter.SPI_cs, LOW);
		#ifdef BME280_useHWSPI
		SPI.transfer(BME280_PRESSURE_MSB | 0x80);		
		
		row_P = (uint32_t)SPI.transfer(0)  << 12;
		row_P |= (uint32_t)SPI.transfer(0)  << 4;
		row_P |= (SPI.transfer(0)  >> 4 )& 0b00001111;
		row_T = (uint32_t)SPI.transfer(0) << 12;
		row_T |= (uint32_t)SPI.transfer(0) << 4;
		row_T |= (SPI.transfer(0) >> 4 )& 0b00001111;
		row_H = (uint32_t)SPI.transfer(0) << 8;
		row_H |= (uint32_t)SPI.transfer(0);
		#endif
		digitalWrite(parameter.SPI_cs, HIGH);		
	}
	
	else												//I2C (default)
	{
		#ifdef BME280_useI2C
		Wire.beginTransmission(parameter.I2CAddress);
		Wire.write(reg);
		Wire.endTransmission();
		Wire.requestFrom(parameter.I2CAddress,(uint8_t)1);		

		row_P = (uint32_t)Wire.read()  << 12;
		row_P |= (uint32_t)Wire.read()  << 4;
		row_P |= (Wire.read()  >> 4 )& 0b00001111;
		row_T = (uint32_t)Wire.read() << 12;
		row_T |= (uint32_t)Wire.read() << 4;
		row_T |= (Wire.read() >> 4 )& 0b00001111;
		row_H = (uint32_t)Wire.read() << 8;
		row_H |= (uint32_t)Wire.read();
		#endif	
	}
	//calc temp first since it is needed for presure and huminity
	// null-pointer check 
	if (valueT != NULL){
		*valueT = calcTempC(row_T);
	}
	if (valueP != NULL){
		*valueP = calcPressure(row_P);
	}
	if (valueH != NULL){
		*valueH = calcHumidity(row_H);
	}
	return;
}

//##########################################################################
uint8_t BlueDot_BME280::spiTransfer(uint8_t data)		//Software SPI done through bit-banging
{
	uint8_t reply = 0;
	for (int counter = 7; counter >= 0; counter--)
	{
		reply <<= 1;
		digitalWrite(parameter.SPI_sck, LOW);
		if (data || !parameter.spi3)
			digitalWrite(parameter.SPI_mosi, data & (1<<counter));
		else
			pinMode(parameter.SPI_miso, INPUT);			//set to MISO
		delayMicroseconds(SW_SPI_delay_us);
		digitalWrite(parameter.SPI_sck, HIGH);
		if (digitalRead(parameter.SPI_miso))
			reply |= 1;
		delayMicroseconds(SW_SPI_delay_us);
	}
	if (parameter.spi3)									//set to MOSI
		pinMode(parameter.SPI_mosi, OUTPUT);
	return reply;
}