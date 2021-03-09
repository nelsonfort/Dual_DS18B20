

#include <Arduino.h>
#include "OneWire.h"

void OneWire::begin( uint8_t pin )
{
	pin_oneWire = pin;
	pinMode(pin, INPUT);
}


// Perform the oneWire reset function.  We will wait up to 250uS for
// the bus to come high, if it doesn't then it is broken or shorted
// and we return a "pulseNotDetected".
// Returns "functionBusy" if the function is still working.
// MAX BLOCKING TIME PER CALL: 70us
// If the function has finish it returns "pulseNotDetected" or "pulseDetected"
enum_oneWireState OneWire::reset(void)
{
	/*indicates the step on the state machine*/
	static enum{ startToWaitUntilIsHigh, waitUntilIsHigh, driveOutputLow, allowBusToBeFloat, waitToFinish } step = startToWaitUntilIsHigh; 
	
	static bool firstEntry = true; /*used as flag for the entry function of the machine state*/
	enum_oneWireState functionState = functionBusy; /*function return value*/
	
	static uint8_t busReading; /*registers if there is a device asserted a presence pulse*/ 
	static uint8_t retries = 125;

	switch( step )
	{
		case startToWaitUntilIsHigh:
			pinMode(pin_oneWire, INPUT);
			delayMicros.start(2);
			step = waitUntilIsHigh; /*goes to the next step*/
			break;
	
		case waitUntilIsHigh: // waits until the wire is high... just in case
			if( delayMicros.justFinished() )
			{
				if( digitalRead(pin_oneWire) )
					step = driveOutputLow; /*goes to the next step*/
			}
			else
			{	
				delayMicros.start(2);
				if( --retries == 0)
				{
					functionState = pulseNotDetected;
					step = startToWaitUntilIsHigh; /*resets the steps*/
				}
			}
			break;
			
		case driveOutputLow:
			if( firstEntry ) /*if the state machine is entering the state*/
			{
				firstEntry = false;
				pinMode(pin_oneWire, OUTPUT);
				digitalWrite(pin_oneWire, LOW);
				delayMicros.start(480);
			}
			else if(delayMicros.justFinished())
			{
				firstEntry = true;
				step = allowBusToBeFloat; /*goes to the next step*/
			}	
			break;
			
		case allowBusToBeFloat:
			pinMode(pin_oneWire, INPUT); //allow it to float
			delayMicroseconds(70);
			busReading = digitalRead(pin_oneWire);
			step = waitToFinish; /*goes to the next step*/
			
			break;
			
		case waitToFinish:
			if( firstEntry )
			{
				firstEntry = false;
				delayMicros.start(410);
			}
			else if( delayMicros.justFinished() )
			{
				firstEntry = true;
				if( busReading == 0) /*if there is a presence pulse*/
				{  
					functionState = pulseDetected;
				}
				else
				{
					functionState = pulseNotDetected;
				}
				step = startToWaitUntilIsHigh; /*resets the steps, so goes to the first step*/	
			}
			break;
	}
	
	return functionState;
}

// Write a bit. Port and bit is used to cut lookup time and provide
// more certain timing.
// MAX BLOCKING TIME PER CALL: 70us
void OneWire::write_bit(uint8_t v)
{
	static uint8_t firstDelay = 0;
	static uint8_t secondDelay = 0;
	
	noInterrupts();
	
	/*checks bit to write*/
	if(v & 1)
	{
		firstDelay = 10;
		secondDelay = 55;
	}
	else
	{
		firstDelay = 65;
		secondDelay = 5;
	}
	
	/*drives output LOW*/
	pinMode(pin_oneWire, OUTPUT);
	digitalWrite(pin_oneWire, LOW);
	delayMicroseconds(firstDelay);
	
	/*drives output HIGH*/
	digitalWrite(pin_oneWire, HIGH);
	delayMicroseconds(secondDelay);
	
	interrupts();
}


// Read a bit. Port and bit is used to cut lookup time and provide
// more certain timing.
// This function has blocking delays to assure the actions be on time
// MAX BLOCKING TIME PER CALL: 66us 
// Returns "bitReadHigh" or "bitReadLow"
enum_oneWireState OneWire::read_bit(void)
{
	enum_oneWireState functionState; /*function return value*/
	static uint8_t busReading = 0;
	
	noInterrupts();

	/*startsReadTimeSlot*/
	pinMode(pin_oneWire, OUTPUT);
	digitalWrite(pin_oneWire, LOW);
	delayMicroseconds(3);
	
	/*letPinFloat*/
	pinMode(pin_oneWire, INPUT);
	delayMicroseconds(10);
	
	/*readBit*/
	busReading = digitalRead(pin_oneWire);//DIRECT_READ(reg, mask);
	if(busReading != 0)
		functionState = bitReadHigh;
	else
		functionState = bitReadLow;
	
	delayMicroseconds(53);
	
	interrupts();

	return functionState;
}

// Write a byte. The writing code uses the active drivers to raise the
// pin high, if you need power after the write (e.g. DS18S20 in
// parasite power mode) then set 'power' to 1, otherwise the pin will
// go tri-state at the end of the write to avoid heating in a short or
// other mishap.
// MAX BLOCKING TIME PER CALL: 70us
// Returns "functionBusy" or "functionFinishes"
enum_oneWireState OneWire::write(uint8_t v, uint8_t power /* = 0 */) 
{

	static uint8_t bitMask = 0x01;
	enum_oneWireState functionState = functionBusy;
	
	
	OneWire::write_bit( (bitMask & v)?1:0);
	bitMask <<= 1;
	
	if( bitMask > 0 )
		functionState = functionBusy;
	else
	{	
		if ( !power) 
			pinMode(pin_oneWire, INPUT_PULLUP);
		
		bitMask = 0x01; //resets the bitMask
		functionState = functionFinishes;
	}
	
	return functionState;

}

//MAX BLOCKING TIME PER CALL: 70us
//Returns "functionBusy" or "functionFinishes" 
enum_oneWireState OneWire::write_bytes(const uint8_t *buf, uint16_t count, bool power /* = 0 */) {
  
	static uint16_t byteNumber = 0; 
	static uint16_t count_internal = 0;
	enum_oneWireState functionState = functionBusy;
  
	if( byteNumber == 0)  /*on the first machine state step the count is copied to internal variable*/
		count_internal = count;
	
	if( byteNumber < count_internal)
	{
		if(write(buf[byteNumber]) == functionFinishes)
			byteNumber++;
	}
	else
	{
		byteNumber = 0; /*resets the state of the state machine*/
		functionState = functionFinishes;
	}
	
	if (!power)
		pinMode(pin_oneWire, INPUT_PULLUP);
	
	return functionState;
}


// Read a byte
//MAX BLOCKING TIME PER CALL: 66us
// Returns "functionBusy", "functionFinishes"
enum_oneWireState OneWire::read( uint8_t *readBuffer) {
	enum_oneWireState functionState = functionBusy;
	static uint8_t bitMask = 0x01;
	static uint8_t readBuffer_internal = 0;
	
	
	if( (OneWire::read_bit() == bitReadHigh) ) /*if the bit is read LOW, the function left the 0 and goes to the next bit*/
		readBuffer_internal |= bitMask;
	
	bitMask <<= 1; /*shifts the bitMask to write the next buffer bit*/
	
	if( bitMask == 0 ) /*when the shift process is over*/
	{
		functionState = functionFinishes;
		bitMask = 0x01; /*resets the bitMask*/
		*readBuffer = readBuffer_internal; /*copies the reading into the buffer*/
		readBuffer_internal = 0; /*cleans the buffer*/
	}
	
	return functionState;
}


//MAX BLOCKING TIME PER CALL: 66us
//Returns "functionBusy" or "functionFinishes"
enum_oneWireState OneWire::read_bytes(uint8_t *buf, uint16_t count) {

	static uint16_t byteNumber = 0;
	static uint16_t count_internal = 0;
	enum_oneWireState functionState = functionBusy;
	
	if( byteNumber == 0)	/*on the first machine state step the count is copied to internal variable*/
		count_internal = count;
	
	if( byteNumber < count_internal )
	{
		if( read( &buf[byteNumber] ) == functionFinishes ) /*when it finishes the read function loads into byte buffer*/
			byteNumber++;
	}
	else
	{
		functionState = functionFinishes;
		byteNumber = 0; /*resets the byte number*/
		count_internal = 0;
	}
	
	return functionState;
}


// Do a ROM select
// MAX BLOCKING TIME PER CALL: 70us
// Returns "functionBusy" or "functionFinishes"
enum_oneWireState OneWire::select(const uint8_t rom[8])
{
	static uint8_t numberByteSended = 0;
	enum_oneWireState functionState = functionBusy;
	
	if( numberByteSended == 0 )
	{
		if( write(0x55) == functionFinishes) //Choose ROM
			numberByteSended++;
	}
	else
	{
		if(numberByteSended < 9) /*if the 64 bits it wasn't yet sent*/
		{
			if( write(rom[numberByteSended - 1]) == functionFinishes )
				numberByteSended++;
		}
		else /*if the 64 bits of ROM was sent*/
		{
			numberByteSended = 0; /*resets the internal variable*/
			functionState = functionFinishes;
		}
	}
	
	return functionState;
}

// Do a ROM skip
// MAX BLOCKING TIME PER CALL: 70us
// Returns "functionBusy" or "functionFinishes"
enum_oneWireState OneWire::skip()
{
	enum_oneWireState functionState = functionBusy;
	
	if(write(0xCC) == functionFinishes) /*Skip ROM*/
		functionState = functionFinishes;
	
	return functionState;
}
