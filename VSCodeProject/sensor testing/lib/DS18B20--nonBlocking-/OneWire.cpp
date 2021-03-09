/*
Copyright (c) 2007, Jim Studt  (original old version - many contributors since)

The latest version of this library may be found at:
  http://www.pjrc.com/teensy/td_libs_OneWire.html

OneWire has been maintained by Paul Stoffregen (paul@pjrc.com) since
January 2010.

DO NOT EMAIL for technical support, especially not for ESP chips!
All project support questions must be posted on public forums
relevant to the board or chips used.  If using Arduino, post on
Arduino's forum.  If using ESP, post on the ESP community forums.
There is ABSOLUTELY NO TECH SUPPORT BY PRIVATE EMAIL!

Github's issue tracker for OneWire should be used only to report
specific bugs.  DO NOT request project support via Github.  All
project and tech support questions must be posted on forums, not
github issues.  If you experience a problem and you are not
absolutely sure it's an issue with the library, ask on a forum
first.  Only use github to report issues after experts have
confirmed the issue is with OneWire rather than your project.

Back in 2010, OneWire was in need of many bug fixes, but had
been abandoned the original author (Jim Studt).  None of the known
contributors were interested in maintaining OneWire.  Paul typically
works on OneWire every 6 to 12 months.  Patches usually wait that
long.  If anyone is interested in more actively maintaining OneWire,
please contact Paul (this is pretty much the only reason to use
private email about OneWire).

OneWire is now very mature code.  No changes other than adding
definitions for newer hardware support are anticipated.

Version 2.3:
  Unknown chip fallback mode, Roger Clark
  Teensy-LC compatibility, Paul Stoffregen
  Search bug fix, Love Nystrom

Version 2.2:
  Teensy 3.0 compatibility, Paul Stoffregen, paul@pjrc.com
  Arduino Due compatibility, http://arduino.cc/forum/index.php?topic=141030
  Fix DS18B20 example negative temperature
  Fix DS18B20 example's low res modes, Ken Butcher
  Improve reset timing, Mark Tillotson
  Add const qualifiers, Bertrik Sikken
  Add initial value input to crc16, Bertrik Sikken
  Add target_search() function, Scott Roberts

Version 2.1:
  Arduino 1.0 compatibility, Paul Stoffregen
  Improve temperature example, Paul Stoffregen
  DS250x_PROM example, Guillermo Lovato
  PIC32 (chipKit) compatibility, Jason Dangel, dangel.jason AT gmail.com
  Improvements from Glenn Trewitt:
  - crc16() now works
  - check_crc16() does all of calculation/checking work.
  - Added read_bytes() and write_bytes(), to reduce tedious loops.
  - Added ds2408 example.
  Delete very old, out-of-date readme file (info is here)

Version 2.0: Modifications by Paul Stoffregen, January 2010:
http://www.pjrc.com/teensy/td_libs_OneWire.html
  Search fix from Robin James
    http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1238032295/27#27
  Use direct optimized I/O in all cases
  Disable interrupts during timing critical sections
    (this solves many random communication errors)
  Disable interrupts during read-modify-write I/O
  Reduce RAM consumption by eliminating unnecessary
    variables and trimming many to 8 bits
  Optimize both crc8 - table version moved to flash

Modified to work with larger numbers of devices - avoids loop.
Tested in Arduino 11 alpha with 12 sensors.
26 Sept 2008 -- Robin James
http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1238032295/27#27

Updated to work with arduino-0008 and to include skip() as of
2007/07/06. --RJL20

Modified to calculate the 8-bit CRC directly, avoiding the need for
the 256-byte lookup table to be loaded in RAM.  Tested in arduino-0010
-- Tom Pollard, Jan 23, 2008

Jim Studt's original library was modified by Josh Larios.

Tom Pollard, pollard@alum.mit.edu, contributed around May 20, 2008

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

Much of the code was inspired by Derek Yerger's code, though I don't
think much of that remains.  In any event that was..
    (copyleft) 2006 by Derek Yerger - Free to distribute freely.

The CRC code was excerpted and inspired by the Dallas Semiconductor
sample code bearing this copyright.
//---------------------------------------------------------------------------
// Copyright (C) 2000 Dallas Semiconductor Corporation, All Rights Reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY,  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL DALLAS SEMICONDUCTOR BE LIABLE FOR ANY CLAIM, DAMAGES
// OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
// ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
// OTHER DEALINGS IN THE SOFTWARE.
//
// Except as contained in this notice, the name of Dallas Semiconductor
// shall not be used except as stated in the Dallas Semiconductor
// Branding Policy.
//--------------------------------------------------------------------------
*/

#include <Arduino.h>
#include "OneWire.h"
#include "util/OneWire_direct_gpio.h"



void OneWire::begin( uint8_t pin )
{
	pin_oneWire = pin;
	pinMode(pin, INPUT);
#if ONEWIRE_SEARCH
	reset_search();
#endif
}


//
// Perform the oneWire reset function.  We will wait up to 250uS for
// the bus to come high, if it doesn't then it is broken or shorted
// and we return a "pulseNotDetected".
// Returns "functionBusy" if the function is still working.
//
// MAX BLOCKING TIME PER CALL: 70us
// If the function has finish it returns "pulseNotDetected" or "pulseDetected"
//
enum_oneWireState OneWire::reset(void)
{
	/*BEGINS NEW CODE*/
	/*indicates the step on the state machine*/
	static enum{ startToWaitUntilIsHigh, waitUntilIsHigh, driveOutputLow, allowBusToBeFloat, waitToFinish } step = startToWaitUntilIsHigh; 
	
	static bool firstEntry = true; /*used as flag for the entry function of the machine state*/
	enum_oneWireState functionState = functionBusy; /*function return value*/
	/*ENDS NEW CODE*/
	
	static uint8_t busReading; /*registers if there is a device asserted a presence pulse*/ 
	static uint8_t retries = 125;

	//**BEGINS NEW CODE*//
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
	//**END NEW CODE**//
	
	//*BEGINS OLD CODE*//
	// wait until the wire is high... just in case
	/*do {
		if (--retries == 0) return 0;
		delayMicroseconds(2);
	} while ( !DIRECT_READ(reg, mask));

	noInterrupts();
	DIRECT_WRITE_LOW(reg, mask);
	DIRECT_MODE_OUTPUT(reg, mask);	// drive output low
	interrupts();
	delayMicroseconds(480);
	noInterrupts();
	DIRECT_MODE_INPUT(reg, mask);	// allow it to float
	delayMicroseconds(70);
	r = !DIRECT_READ(reg, mask);
	interrupts();
	delayMicroseconds(410);
	return r;*/
	//*ENDS OLD CODE*//
}


//
// Write a bit. Port and bit is used to cut lookup time and provide
// more certain timing.
// MAX BLOCKING TIME PER CALL: 70us
//
void OneWire::write_bit(uint8_t v)
{
	/*BEGINS NEW CODE*/
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
	
	//*ENDS NEW CODE*//
	
	//*BEGINS OLD CODE*//
	/*if (v & 1) {
		noInterrupts();
		DIRECT_WRITE_LOW(reg, mask);
		DIRECT_MODE_OUTPUT(reg, mask);	// drive output low
		delayMicroseconds(10);
		DIRECT_WRITE_HIGH(reg, mask);	// drive output high
		interrupts();
		delayMicroseconds(55);
	} else {
		noInterrupts();
		DIRECT_WRITE_LOW(reg, mask);
		DIRECT_MODE_OUTPUT(reg, mask);	// drive output low
		delayMicroseconds(65);
		DIRECT_WRITE_HIGH(reg, mask);	// drive output high
		interrupts();
		delayMicroseconds(5);
	}*/
	//*ENDS OLD CODE*//
}


//
// Read a bit. Port and bit is used to cut lookup time and provide
// more certain timing.
// This function has blocking delays to assure the actions be on time
// MAX BLOCKING TIME PER CALL: 66us 
//
// Returns "bitReadHigh" or "bitReadLow"
//
enum_oneWireState OneWire::read_bit(void)
{
	/*BEGINS NEW CODE*/
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
	
	//*ENDS NEW CODE*//
	
	//*BEGINS OLD CODE*//
	/*noInterrupts();
	DIRECT_MODE_OUTPUT(reg, mask);
	DIRECT_WRITE_LOW(reg, mask); //starts a read time Slot
	delayMicroseconds(3);
	DIRECT_MODE_INPUT(reg, mask);	// let pin float, pull up will raise
	delayMicroseconds(10);
	r = DIRECT_READ(reg, mask);
	interrupts();
	delayMicroseconds(53);
	return r;*/
	//*ENDS OLD CODE*//
}

//
// Write a byte. The writing code uses the active drivers to raise the
// pin high, if you need power after the write (e.g. DS18S20 in
// parasite power mode) then set 'power' to 1, otherwise the pin will
// go tri-state at the end of the write to avoid heating in a short or
// other mishap.
//
// MAX BLOCKING TIME PER CALL: 70us
// Returns "functionBusy" or "functionFinishes"
enum_oneWireState OneWire::write(uint8_t v, uint8_t power /* = 0 */) {
    
	//*BEGINS NEW CODE*//
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
	//*ENDS NEW CODE*//
	
	//*BEGINS OLD CODE*//
	/*uint8_t bitMask;

    for (bitMask = 0x01; bitMask ; bitMask <<= 1) {
	OneWire::write_bit( (bitMask & v)?1:0);
    }
    if ( !power) {
	noInterrupts();
	DIRECT_MODE_INPUT(baseReg, bitmask);
	DIRECT_WRITE_LOW(baseReg, bitmask);
	interrupts();
    }*/
	//*ENDS OLD CODE*//
}


//MAX BLOCKING TIME PER CALL: 70us
//Returns "functionBusy" or "functionFinishes" 
enum_oneWireState OneWire::write_bytes(const uint8_t *buf, uint16_t count, bool power /* = 0 */) {
  
	//*BEGINS NEW CODE*//
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
	//*ENDS NEW CODE*//
  
	//*BEGINS OLD CODE*//
	/*for (uint16_t i = 0 ; i < count ; i++)
		write(buf[i]);
	if (!power) 
	{
		noInterrupts();
		DIRECT_MODE_INPUT(baseReg, bitmask);
		DIRECT_WRITE_LOW(baseReg, bitmask);
		interrupts();
	}*/
	//*ENDS OLD CODE*//
}


//
// Read a byte
//MAX BLOCKING TIME PER CALL: 66us
//
// Returns "functionBusy", "functionFinishes"
//*********************************CORREGIR***********************************************************////
enum_oneWireState OneWire::read( uint8_t *readBuffer) {
    //*BEGINS NEW CODE*//
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
	//*ENDS NEW CODE*//
	
	//*BEGINS OLD CODE*//
	/*uint8_t bitMask;
    uint8_t r = 0;

    for (bitMask = 0x01; bitMask; bitMask <<= 1) {
	if ( OneWire::read_bit()) r |= bitMask;
    }
    return r;*/
	//*ENDS OLD CODE*//
}


//MAX BLOCKING TIME PER CALL: 66us
//Returns "functionBusy" or "functionFinishes"
enum_oneWireState OneWire::read_bytes(uint8_t *buf, uint16_t count) {
	
	//*BEGINS NEW CODE*//
	
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
	//*ENDS NEW CODE*//
	
	//*BEGINS OLD CODE*//
	/*for (uint16_t i = 0 ; i < count ; i++)
		buf[i] = read();*/
	//*ENDS OLD CODE*//
}


//
// Do a ROM select
// MAX BLOCKING TIME PER CALL: 70us
// Returns "functionBusy" or "functionFinishes"
enum_oneWireState OneWire::select(const uint8_t rom[8])
{
    //*BEGINS NEW CODE*//
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
	//*ENDS NEW CODE*//
	
	//*BEGINS OLD CODE*//
	/*uint8_t i;

    write(0x55);           // Choose ROM

    for (i = 0; i < 8; i++) write(rom[i]);*/
	//*ENDS OLD CODE*//
}

//
// Do a ROM skip
// MAX BLOCKING TIME PER CALL: 70us
// Returns "functionBusy" or "functionFinishes"
enum_oneWireState OneWire::skip()
{
    //*BEGINS NEW CODE*//
	
	enum_oneWireState functionState = functionBusy;
	
	if(write(0xCC) == functionFinishes) /*Skip ROM*/
		functionState = functionFinishes;
	
	return functionState;
	//*ENDS NEW CODE*//
	
	//*BEGINS OLD CODE*//
	//write(0xCC);           // Skip ROM
	//*ENDS OLD CODE*//
}

void OneWire::depower()
{
	noInterrupts();
	DIRECT_MODE_INPUT(baseReg, bitmask);
	interrupts();
}

#if ONEWIRE_SEARCH

//
// You need to use this function to start a search again from the beginning.
// You do not need to do it for the first search, though you could.
//
void OneWire::reset_search()
{
  // reset the search state
  LastDiscrepancy = 0;
  LastDeviceFlag = false;
  LastFamilyDiscrepancy = 0;
  for(int i = 7; ; i--) {
    ROM_NO[i] = 0;
    if ( i == 0) break;
  }
}

// Setup the search to find the device type 'family_code' on the next call
// to search(*newAddr) if it is present.
//
void OneWire::target_search(uint8_t family_code)
{
   // set the search state to find SearchFamily type devices
   ROM_NO[0] = family_code;
   for (uint8_t i = 1; i < 8; i++)
      ROM_NO[i] = 0;
   LastDiscrepancy = 64;
   LastFamilyDiscrepancy = 0;
   LastDeviceFlag = false;
}

//
// Perform a search. If this function returns a '1' then it has
// enumerated the next device and you may retrieve the ROM from the
// OneWire::address variable. If there are no devices, no further
// devices, or something horrible happens in the middle of the
// enumeration then a 0 is returned.  If a new device is found then
// its address is copied to newAddr.  Use OneWire::reset_search() to
// start over.
//
// --- Replaced by the one from the Dallas Semiconductor web site ---
//--------------------------------------------------------------------------
// Perform the 1-Wire Search Algorithm on the 1-Wire bus using the existing
// search state.
// Return TRUE  : device found, ROM number in ROM_NO buffer
//        FALSE : device not found, end of search
//
bool OneWire::search(uint8_t *newAddr, bool search_mode /* = true */)
{
   uint8_t id_bit_number;
   uint8_t last_zero, rom_byte_number;
   bool    search_result;
   uint8_t id_bit, cmp_id_bit;

   unsigned char rom_byte_mask, search_direction;

   // initialize for search
   id_bit_number = 1;
   last_zero = 0;
   rom_byte_number = 0;
   rom_byte_mask = 1;
   search_result = false;

   // if the last call was not the last one
   if (!LastDeviceFlag) {
      // 1-Wire reset
      if (!reset()) {
         // reset the search
         LastDiscrepancy = 0;
         LastDeviceFlag = false;
         LastFamilyDiscrepancy = 0;
         return false;
      }

      // issue the search command
      if (search_mode == true) {
        write(0xF0);   // NORMAL SEARCH
      } else {
        write(0xEC);   // CONDITIONAL SEARCH
      }

      // loop to do the search
      do
      {
         // read a bit and its complement
         id_bit = read_bit();
         cmp_id_bit = read_bit();

         // check for no devices on 1-wire
         if ((id_bit == 1) && (cmp_id_bit == 1)) {
            break;
         } else {
            // all devices coupled have 0 or 1
            if (id_bit != cmp_id_bit) {
               search_direction = id_bit;  // bit write value for search
            } else {
               // if this discrepancy if before the Last Discrepancy
               // on a previous next then pick the same as last time
               if (id_bit_number < LastDiscrepancy) {
                  search_direction = ((ROM_NO[rom_byte_number] & rom_byte_mask) > 0);
               } else {
                  // if equal to last pick 1, if not then pick 0
                  search_direction = (id_bit_number == LastDiscrepancy);
               }
               // if 0 was picked then record its position in LastZero
               if (search_direction == 0) {
                  last_zero = id_bit_number;

                  // check for Last discrepancy in family
                  if (last_zero < 9)
                     LastFamilyDiscrepancy = last_zero;
               }
            }

            // set or clear the bit in the ROM byte rom_byte_number
            // with mask rom_byte_mask
            if (search_direction == 1)
              ROM_NO[rom_byte_number] |= rom_byte_mask;
            else
              ROM_NO[rom_byte_number] &= ~rom_byte_mask;

            // serial number search direction write bit
            write_bit(search_direction);

            // increment the byte counter id_bit_number
            // and shift the mask rom_byte_mask
            id_bit_number++;
            rom_byte_mask <<= 1;

            // if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
            if (rom_byte_mask == 0) {
                rom_byte_number++;
                rom_byte_mask = 1;
            }
         }
      }
      while(rom_byte_number < 8);  // loop until through all ROM bytes 0-7

      // if the search was successful then
      if (!(id_bit_number < 65)) {
         // search successful so set LastDiscrepancy,LastDeviceFlag,search_result
         LastDiscrepancy = last_zero;

         // check for last device
         if (LastDiscrepancy == 0) {
            LastDeviceFlag = true;
         }
         search_result = true;
      }
   }

   // if no device found then reset counters so next 'search' will be like a first
   if (!search_result || !ROM_NO[0]) {
      LastDiscrepancy = 0;
      LastDeviceFlag = false;
      LastFamilyDiscrepancy = 0;
      search_result = false;
   } else {
      for (int i = 0; i < 8; i++) newAddr[i] = ROM_NO[i];
   }
   return search_result;
  }

#endif

#if ONEWIRE_CRC
// The 1-Wire CRC scheme is described in Maxim Application Note 27:
// "Understanding and Using Cyclic Redundancy Checks with Maxim iButton Products"
//

#if ONEWIRE_CRC8_TABLE
// Dow-CRC using polynomial X^8 + X^5 + X^4 + X^0
// Tiny 2x16 entry CRC table created by Arjen Lentz
// See http://lentz.com.au/blog/calculating-crc-with-a-tiny-32-entry-lookup-table
static const uint8_t PROGMEM dscrc2x16_table[] = {
	0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83,
	0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
	0x00, 0x9D, 0x23, 0xBE, 0x46, 0xDB, 0x65, 0xF8,
	0x8C, 0x11, 0xAF, 0x32, 0xCA, 0x57, 0xE9, 0x74
};

// Compute a Dallas Semiconductor 8 bit CRC. These show up in the ROM
// and the registers.  (Use tiny 2x16 entry CRC table)
uint8_t OneWire::crc8(const uint8_t *addr, uint8_t len)
{
	uint8_t crc = 0;

	while (len--) {
		crc = *addr++ ^ crc;  // just re-using crc as intermediate
		crc = pgm_read_byte(dscrc2x16_table + (crc & 0x0f)) ^
		pgm_read_byte(dscrc2x16_table + 16 + ((crc >> 4) & 0x0f));
	}

	return crc;
}
#else
//
// Compute a Dallas Semiconductor 8 bit CRC directly.
// this is much slower, but a little smaller, than the lookup table.
//
uint8_t OneWire::crc8(const uint8_t *addr, uint8_t len)
{
	uint8_t crc = 0;

	while (len--) {
#if defined(__AVR__)
		crc = _crc_ibutton_update(crc, *addr++);
#else
		uint8_t inbyte = *addr++;
		for (uint8_t i = 8; i; i--) {
			uint8_t mix = (crc ^ inbyte) & 0x01;
			crc >>= 1;
			if (mix) crc ^= 0x8C;
			inbyte >>= 1;
		}
#endif
	}
	return crc;
}
#endif

#if ONEWIRE_CRC16
bool OneWire::check_crc16(const uint8_t* input, uint16_t len, const uint8_t* inverted_crc, uint16_t crc)
{
    crc = ~crc16(input, len, crc);
    return (crc & 0xFF) == inverted_crc[0] && (crc >> 8) == inverted_crc[1];
}

uint16_t OneWire::crc16(const uint8_t* input, uint16_t len, uint16_t crc)
{
#if defined(__AVR__)
    for (uint16_t i = 0 ; i < len ; i++) {
        crc = _crc16_update(crc, input[i]);
    }
#else
    static const uint8_t oddparity[16] =
        { 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0 };

    for (uint16_t i = 0 ; i < len ; i++) {
      // Even though we're just copying a byte from the input,
      // we'll be doing 16-bit computation with it.
      uint16_t cdata = input[i];
      cdata = (cdata ^ crc) & 0xff;
      crc >>= 8;

      if (oddparity[cdata & 0x0F] ^ oddparity[cdata >> 4])
          crc ^= 0xC001;

      cdata <<= 6;
      crc ^= cdata;
      cdata <<= 1;
      crc ^= cdata;
    }
#endif
    return crc;
}
#endif

#endif
