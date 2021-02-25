/*DS18B20 temperature sensor library.
 * - This library is a non-blocking library.
 * - MAX BLOCKING TIME PER FUNCTION CALL: 70us 
 * - This library is robust to blocking delays between functions calls. MAX BLOCKING DELAY BETWEEN CALLS: unlimited
 * - IMPORTANT: This library only supports 1 sensor connected to each pin simultaneously.
 */
#include "ds18b20.h"

#define BYTE 8
#define NIBBLE 4



/*Publics Methods ------------------------------------------------------------------------------*/

void ds18b20::begin( uint8_t pin ) /*ds18b20 constructor*/
{
	pin_oneWire = pin;
	sensor.begin(pin_oneWire);
}



enum_oneWireState ds18b20::takeMeasure(float *readBuffer)
{
	static enum
	{ 
		firstReset, firstSkip, convertTemperature, waitConversion, 
		secondReset, secondSkip, requestRead, readResult, lastReset
	
	} step = firstReset; /*machine state steps*/
	
	enum_oneWireState functionState = functionBusy;
	enum_oneWireState functionStateBuffer = functionBusy; /*it's used with the reset() that has 3 states*/
	
	switch(step)
	{
		case firstReset:
			functionStateBuffer = sensor.reset();
			/*checks the value returned by sensor.reset()*/
			if( functionStateBuffer == pulseDetected )
				step = firstSkip;
			else if ( functionStateBuffer == pulseNotDetected )
				functionState = pulseNotDetected; /*returns the error condition*/
			break;
		
		case firstSkip:
			if( sensor.skip() == functionFinishes )
				step = convertTemperature;
			break;
			
		case convertTemperature:
			if( convert_T() == functionFinishes )
				step = waitConversion;
			break;
			
		case waitConversion:
			if( waitConversionReady() == functionFinishes )
				step = secondReset;
			break;
			
		case secondReset:
			functionStateBuffer = sensor.reset();
			/*checks the value returned by sensor.reset()*/
			if( functionStateBuffer == pulseDetected )
				step = secondSkip;
			else if ( functionStateBuffer == pulseNotDetected )
			{
				/*returns the error condition and reset the whole operation*/
				functionState = pulseNotDetected;
				step = firstReset;
			}
			break;
			
		case secondSkip:
			if( sensor.skip() == functionFinishes )
				step = requestRead;
			break;
		
		case requestRead:
			if( read_Scratchpad() == functionFinishes )
				step = readResult;
			break;
		
		case readResult:
			if( readTemperature(readBuffer) == functionFinishes )
				step = lastReset;
			break;
			
		case lastReset:
			functionStateBuffer = sensor.reset();
			/*checks if sensor.reset() has finished*/
			if( functionStateBuffer != functionBusy ) 
			{
				functionState = functionFinishes;
				step = firstReset; /*resets the state machine and whole operation*/
			}
			break;
	}

	return functionState;	
}



/*Private Methods ------------------------------------------------------------------------------*/

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
	static uint8_t readBytes[2] = { 0 , 0 };
	
	if( sensor.read_bytes(readBytes, 2) == functionFinishes )
	{
		/*loads the readBuffer with the temperature data*/
		*readBuffer = receivedData_to_float( readBytes, 2 );
		
		/*cleans the readBytes array*/
		readBytes[0] = 0;
		readBytes[1] = 0;
		
		/*indicates the function has finished*/
		functionState = functionFinishes;
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