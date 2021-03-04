/*DS18B20 temperature sensor library.
 * - This library is a non-blocking library.
 * - MAX BLOCKING TIME PER FUNCTION CALL: 70us 
 * - This library is robust to blocking delays between functions calls. MAX BLOCKING DELAY BETWEEN CALLS: unlimited
 * - IMPORTANT: This library only supports 1 sensor connected to each pin simultaneously.
 */
#include "ds18b20.h"

#define BYTE 8
#define NIBBLE 4
#define CRC_PASS true
#define CRC_ERROR false


/*Publics Methods ------------------------------------------------------------------------------*/

void ds18b20::begin( uint8_t pin ) /*ds18b20 constructor*/
{
	pin_oneWire = pin;
	sensor.begin(pin_oneWire);
}



enum_oneWireState ds18b20::takeMeasure(float *readBuffer)
{
	static enum enum_step
	{ 
		convertTemperature, 
		readTemperature
	} step = convertTemperature; /*machine state steps*/
	
	enum_oneWireState returnState = functionBusy; /*it's used as a return value*/
	enum_oneWireState functionState = functionBusy; 
	const enum_step resetStep = convertTemperature;

	switch(step)
	{
		case convertTemperature:
			functionState = convert_T_block();
			if( functionState == functionFinishes )
			{	
				step = readTemperature;
				//DEBUG
					digitalWrite(19, HIGH);
					digitalWrite(19, LOW);
				//DEBUG
			}
			else if( functionState == functionError )
					returnState = functionError;
				break;
						
		case readTemperature:
			functionState = readTemperature_block( readBuffer );
			if( functionState == functionFinishes )
			{	
				step = convertTemperature;
				returnState = functionFinishes;
			}
			else if ( functionState == functionError )
				returnState = functionError;
			break;
			
	}

	//resets the state machine
	if( (returnState == functionFinishes) || (returnState == functionError ) )
		step = resetStep;

	return returnState;	
}



/*Private Methods ------------------------------------------------------------------------------*/

//high level functions
enum_oneWireState ds18b20::setMesasureResolution( uint8_t resolutionBits ) /*sets the bit resolution between 9 and 12 bits*/
{
	return functionBusy;
}

enum_oneWireState ds18b20::convert_T_block( void ) /*generates an temperature conversion request, with reset included*/
{
	static enum{ reset, skip, convertTemperature, waitConversion } step = reset;
	enum_oneWireState returnState = functionBusy; /*it's used as a return value*/
	enum_oneWireState functionState = functionBusy;  /*it's used with the reset() that has 3 states*/

	switch(step)
	{
		case reset:
			functionState = sensor.reset(); 
			/*checks the value returned by sensor.reset()*/
			if( functionState == pulseDetected )
				step = skip;
			else if ( functionState == pulseNotDetected )
				returnState = functionError; /*returns the error condition*/
			break;
		
		case skip:
			if( sensor.skip() == functionFinishes )
				step = convertTemperature;
			break;
			
		case convertTemperature:
			if( convert_T() == functionFinishes )
				step = waitConversion;
			break;

		case waitConversion:
			if( waitConversionReady() == functionFinishes )
				returnState = functionFinishes;
			break;
	}

	//resets the state machine
	if( (returnState == functionFinishes) || (returnState == functionError ) )
		step = reset;
	
	return returnState;	
}

enum_oneWireState ds18b20::readTemperature_block( float *readBuffer ) /*reads the temperature starting with a reset*/
{
	static enum{ reset, skip, requestRead, readResult } step = reset;
	enum_oneWireState returnState = functionBusy; /*it's used as a return value*/
	enum_oneWireState functionState = functionBusy;  /*it's used with the reset() that has 3 states*/

	switch(step)
	{
		case reset:
			functionState = sensor.reset();
			/*checks the value returned by sensor.reset()*/
			if( functionState == pulseDetected )
				step = skip;
			else if ( functionState == pulseNotDetected )
				returnState = functionError;
			break;
			
		case skip:
			if( sensor.skip() == functionFinishes )
				step = requestRead;
			break;
		
		case requestRead:
			if( read_Scratchpad() == functionFinishes )
				step = readResult;
			break;
		
		case readResult:
			functionState = readTemperature(readBuffer);
			if( functionState == functionFinishes )
				returnState = functionFinishes;
			else if( functionState == functionError )
				returnState = functionError;
			break;
	}
	
	//resets the state machine
	if( (returnState == functionFinishes) || (returnState == functionError ) )
		step = reset;
	
	return returnState;
}

//low level functions
enum_oneWireState ds18b20::convert_T()
{
	enum_oneWireState functionState = functionBusy;
	
	if( sensor.write(0x44) == functionFinishes )
		functionState = functionFinishes;
	
	return functionState;
}

enum_oneWireState ds18b20::read_Scratchpad(void)
{
	enum_oneWireState functionState = functionBusy;
	
	if( sensor.write(0xBE) == functionFinishes )
		functionState = functionFinishes;
	
	return functionState;
}


enum_oneWireState ds18b20::waitConversionReady()
{
	enum_oneWireState functionState = functionBusy;
	
	if( sensor.read_bit() == bitReadHigh )
		functionState = functionFinishes;
	
	return functionState;
}




enum_oneWireState ds18b20::readTemperature( float *readBuffer )
{
	enum_oneWireState functionState = functionBusy;
	static uint8_t readBytes[9] = { 0 , 0, 0, 0, 0, 0, 0, 0, 0 }; /*value to store internals registers from ds18b20*/
	

	if( sensor.read_bytes(readBytes, 9) == functionFinishes )
	{
		if( checkCRC(readBytes, 9) == CRC_PASS )
		{
			/*loads the readBuffer with the temperature data*/
			*readBuffer = receivedData_to_float( readBytes, 2 );
						
			/*indicates the function has finished*/
			functionState = functionFinishes;
		}
		else
			functionState = functionError;

		/*cleans the readBytes array*/
			readBytes[0] = 0;
			readBytes[1] = 0;

	}
	
	return functionState;

}




float ds18b20::receivedData_to_float( uint8_t *bytes, uint8_t byteCount )
{
	float convertedValue = 0;
	uint8_t aux = 0;
	
	/*loads the decimal part of the converted value. Bits 0 to 3 from the received data*/
	if( (bytes[0] & 0x01) != 0 )
		convertedValue = 0.0625;
	
	if( (bytes[0] & 0x02) != 0 )
		convertedValue += 0.125;
	
	if( (bytes[0] & 0x04) != 0 )
		convertedValue += 0.25;
	
	if( (bytes[0] & 0x08) != 0 )
		convertedValue += 0.5;
	
	/*loads the integer part of the converted value. Bits 4 to 11 from the received data*/
	aux = bytes[1] << NIBBLE; /*loads the MSF NIBBLE*/
	aux |= (bytes[0] >> NIBBLE) & (0x0F); /*loads the LSF NIBBLE*/
	
	convertedValue += (float) aux;
	
	return convertedValue;
}


bool ds18b20::checkCRC( uint8_t *bytes, uint8_t byteCount) /*checks the CRC and returns CRC_PASS or CRC_ERROR*/
{
	/*CRC calculation:
	 * CRC generator:   
	 *  bit_7_new = bit_0_old XOR bit_input 
	 *  bit_6_new = bit_7_old  
	 *  bit_5_new = bit_6_old
	 *  bit_4_new = bit_5_old
	 *  bit_3_new = bit_4_old XOR bit_7_new
	 *  bit_2_new = bit_3_old XOR bit_7_new 
	 *  bit_1_new = bit_2_old
	 *  bit_0_new = bit_1_old*/
	
	/*counter*/
	int bitNumber = 0;
	uint8_t byteNumber = 0; /*used to adress the byte on the uint8 array*/

	/*variables to store data on processing*/
	uint8_t bit_input = 0;
	uint8_t bit_7 = 0;
	uint8_t bit_7_old = 0;

	uint8_t bit_6 = 0;
	uint8_t bit_6_old = 0;

	uint8_t bit_5 = 0;
	uint8_t bit_5_old = 0;

	uint8_t bit_4 = 0;
	uint8_t bit_4_old = 0;

	uint8_t bit_3 = 0;
	uint8_t bit_3_old = 0;

	uint8_t bit_2 = 0;
	uint8_t bit_2_old = 0;

	uint8_t bit_1 = 0;
	uint8_t bit_1_old = 0;


	uint8_t bit_0 = 0;
	uint8_t bit_0_old = 0;

	/*CRC results*/
	bool returnValue = CRC_ERROR;
	uint8_t crcCalculated = 0; /*stores the rest to be compare with the last byte received from the sensor*/


	if( byteCount == 9 )
	{
		for (bitNumber = 0; bitNumber < 72; bitNumber++)
		{
			/*takes the next input bit*/
			byteNumber = bitNumber / 8;
			bit_input = ( bytes[byteNumber] >> (bitNumber % 8) ) & 0x01; /*takes only the current bit*/
			
			bit_7 = (bit_0_old ^ bit_input) & 0x01;
			bit_6 = (bit_7_old) & 0x01;
			bit_5 = bit_6_old & 0x01;
			bit_4 = bit_5_old & 0x01;
			bit_3 = (bit_4_old ^ bit_7) & 0x01;
			bit_2 = (bit_3_old ^ bit_7) & 0x01;
			bit_1 = bit_2_old & 0x01;
			bit_0 = bit_1_old & 0x01;

			/*updates old bits*/
			bit_7_old = bit_7;
			bit_6_old = bit_6;
			bit_5_old = bit_5;
			bit_4_old = bit_4;
			bit_3_old = bit_3;
			bit_2_old = bit_2;
			bit_1_old = bit_1;
			bit_0_old = bit_0;
		
		}

		/*after the bit calculation constructs the CRC byte*/
		crcCalculated = (bit_7 << 7) + (bit_6 << 6) + (bit_5 << 5) + (bit_4 << 4);
		crcCalculated += (bit_3 << 3) + (bit_2 << 2) + (bit_1 << 1) + bit_0;

		/*compares the crc calculated with the */
		if( crcCalculated == 0 )
			returnValue = CRC_PASS;
		else
			returnValue = CRC_ERROR;

	}

	return returnValue;
}