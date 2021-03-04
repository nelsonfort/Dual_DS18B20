#ifndef OneWire_h
#define OneWire_h

#ifdef __cplusplus

#include <stdint.h>

#if defined(__AVR__)
#include <util/crc16.h>     ///< used to verify the data received from the sensor
#endif

#if ARDUINO >= 100
#include <Arduino.h>       ///< for delayMicroseconds, digitalPinToBitMask, etc
#else
#include "WProgram.h"      ///< for delayMicroseconds
#include "pins_arduino.h"  ///< for digitalPinToBitMask, etc
#endif

/** You can exclude certain features from OneWire.  
 * - In theory, this might save some space.  
 * - In practice, the compiler automatically removes unused code 
 *   (technically, the linker, using -fdata-sections and -ffunction-sections when compiling, and Wl,--gc-sections
* when linking).
* .
* So most of these will not result in any code size reduction.  
* Well, unless you try to use the missing features
* and redesign your program to not need them!  ONEWIRE_CRC8_TABLE
* is the exception, because it selects a fast but large algorithm
* or a small but slow algorithm.
* - You can exclude onewire_search by defining that to 0.
* - You can exclude CRC checks altogether by defining this to 0.
* .
*/

#ifndef ONEWIRE_SEARCH
#define ONEWIRE_SEARCH 1
#endif

#ifndef ONEWIRE_CRC
#define ONEWIRE_CRC 1
#endif

/** Select the table-lookup method of computing the 8-bit CRC
* by setting this to 1.  The lookup table enlarges code size by
* about 250 bytes.  It does NOT consume RAM (but did in very
* old versions of OneWire).  If you disable this, a slower
* but very compact algorithm is used.
*/
#ifndef ONEWIRE_CRC8_TABLE
#define ONEWIRE_CRC8_TABLE 1
#endif

/* You can allow 16-bit CRC checks by defining this to 1
* (Note that ONEWIRE_CRC must also be 1.)
*/
#ifndef ONEWIRE_CRC16
#define ONEWIRE_CRC16 1
#endif

/* This library contains Board-specific macros for direct GPIO
*/
#include "util/OneWire_direct_regtype.h"

#include <microsDelay.h>
/*TypeDefs ----------------------------------------------------------------------------*/
typedef enum
{
	functionBusy,
	pulseDetected,
	pulseNotDetected,
	functionFinishes,
	bitReadHigh,
	bitReadLow,
    functionError
}enum_oneWireState;

/**
 * - OneWire - Class definition
 * @brief *Contains the structure of the class detailing public and private
 *  atributes.
 * And the methods that are used depending the previous configuration made.
 * 
 */
class OneWire
{
  private:
    IO_REG_TYPE bitmask;
    volatile IO_REG_TYPE *baseReg;
	microsDelay delayMicros; /*non blocking delay in us class*/
	uint8_t pin_oneWire;
/**
 * @brief This attributes are added if ONEWIRE_SEARCH is 1.
 * 
 */
#if ONEWIRE_SEARCH
    // global search state
    unsigned char ROM_NO[8];
    uint8_t LastDiscrepancy;
    uint8_t LastFamilyDiscrepancy;
    bool LastDeviceFlag;
#endif

  public:
    /**
     * @brief Construct a new One Wire object (Without input parameters)
     * 
     */
    OneWire() { }
    /**
     * @brief Construct a new One Wire object (With pin parameters)
     * 
     * @param pin Data pin where is connected the sensor.
     */
    OneWire(uint8_t pin) { begin (pin);}

    void begin (uint8_t pin);
    /**
     * @brief Perform a 1-Wire reset cycle. Returns 1 if a device responds
     * with a presence pulse.
     * During the initialization sequence the bus master transmits (TX) 
     * the reset pulse by pulling the 1-Wire bus low for a minimum of 480µs.
     * 
     * @return enum_oneWireState 
     * - Returns "functionBusy" if the function is still working.
     * - If the function has finished, it returns:
     *   - "pulseNotDetected".
     *   - "pulseDetected".
     *   .
     * .
     */
    enum_oneWireState reset(void);

    // Issue a 1-Wire rom select command, you do the reset first.
    /**
     * @brief Select the device using the address stored in the rom
     * 
     * @param rom Address of a device stored in the rom register.
     * @return enum_oneWireState 
     * - Returns "functionBusy" if the function is still working.
     * - If the function has finished, it returns:
     *   - "pulseNotDetected".
     *   - "pulseDetected".
     *   .
     * .
     */
    enum_oneWireState select(const uint8_t rom[8]);

    // Issue a 1-Wire rom skip command, to address all on bus.
    /**
     * @brief This method is particularly easy to use when
     * only have one sensor connected to the OneWire bus.
     * So it's not required to know its address to communicate
     * with it.
     * @return enum_oneWireState 
     * - Returns "functionBusy" if the function is still working.
     * - If the function has finished, it returns:
     *   - "pulseNotDetected".
     *   - "pulseDetected".
     *   .
     * .
     */
    enum_oneWireState skip(void);


    /**
     * @brief Write a byte.
     * 
     * @param v Data to be written in the device.
     * @param power If 'power = 1' then the wire is held high at the end 
     * for parasitically powered devices. 
     * You are responsible for eventually depowering it by calling 
     * depower() or doing another read or write.
    
     * @return enum_oneWireState 
     * - Returns "functionBusy" if the function is still working.
     * - If the function has finished, it returns:
     *   - "pulseNotDetected".
     *   - "pulseDetected".
     *   .
     * .
     */
    enum_oneWireState write(uint8_t v, uint8_t power = 0);
    /**
     * @brief Write multiple bytes to the sensor
     * 
     * @param buf The buffer where is stored the data to be written.
     * @param count Amount of bytes to be written.
     * @param power If 'power = 1' then the wire is held high at the end 
     * for parasitically powered devices. 
     * You are responsible for eventually depowering it by calling 
     * depower() or doing another read or write.
     * @return enum_oneWireState 
     * - Returns "functionBusy" if the function is still working.
     * - If the function has finished, it returns:
     *   - "pulseNotDetected".
     *   - "pulseDetected".
     *   .
     * .
     */
    enum_oneWireState write_bytes(const uint8_t *buf, uint16_t count, bool power = 0);

    /**
     * @brief Reads a byte of data from the sensor.
     * 
     * @param readBuffer The register where will be stored the data received.
     * @return enum_oneWireState 
     * - Returns "functionBusy" if the function is still working.
     * - If the function has finished, it returns:
     *   - "pulseNotDetected".
     *   - "pulseDetected".
     *   .
     * .
     */
    enum_oneWireState read( uint8_t *readBuffer );

    /**
     * @brief This method is used to read multiple bytes from the device
     * 
     * @param buf The received data will be stored in this buffer.
     * @param count The amount of bytes to be read.
     * @return enum_oneWireState 
     * - Returns "functionBusy" if the function is still working.
     * - If the function has finished, it returns:
     *   - "pulseNotDetected".
     *   - "pulseDetected".
     *   .
     * . 
     */
    enum_oneWireState read_bytes(uint8_t *buf, uint16_t count);

    // Write a bit. The bus is always left powered at the end, see
    // note in write() about that.

    /**
     * @brief Write one bit to the sensor.
     * NOTE THAT : The bus is always left powered at the end, see
     * note in write() about that.
     * 
     * @param v Bit to be written in the device
     * @return enum_oneWireState 
     * - Returns "functionBusy" if the function is still working.
     * - If the function has finished, it returns:
     *   - "pulseNotDetected".
     *   - "pulseDetected".
     *   .
     * . 
     */
    void write_bit(uint8_t v);

    // Read a bit.
    /**
     * @brief Read a bit from the device
     * 
     * @return enum_oneWireState 
     * - Returns "functionBusy" if the function is still working.
     * - If the function has finished, it returns:
     *   - "pulseNotDetected".
     *   - "pulseDetected".
     *   - "bitReadHigh" The data that has been read is 1.
	 *   - "bitReadLow" The data that has been read is 0.
     *   .
     * .  
     */
    enum_oneWireState read_bit(void);

    /**
     * @brief Stop forcing power onto the bus. 
     * This method should be used when the 'power' flag in 
     * a write() or write_bit() was set in 1 and aren't about 
     * to do another read or write.
     */
    void depower(void);
/**
 * @brief This block of code will be used if ONEWIRE_SEARCH is 1.
 * 
 */
#if ONEWIRE_SEARCH
    
    /**
     * @brief Clear the search state so that it will start 
     * from the beginning again.
     * 
     */
    void reset_search();


    /**
     * @brief Setup the search to find the device type on the next call
     * to search(*newAddr) if it is present.
     * 
     * @param family_code type of the device that want to be used to search.
     */
    void target_search(uint8_t family_code);

    //  Returns 1 if a new address has been
    // returned. A zero might mean that the bus is shorted, there are
    // no devices, or you have already retrieved all of them.  It
    // might be a good idea to check the CRC to make sure you didn't
    // get garbage.  The order is deterministic. You will always get
    // the same devices in the same order.
    /**
     * @brief Look for the next device.
     * Make sure to check the CRC to avoid problems with a data 
     * transmission corrupted.
     * Always will get the same devices in the same order.
     * @param newAddr 
     * @param search_mode 
     * @return true if a new address has been returned
     *
     * @return false can be received for any of these options: 
     * - The bus is shorted.
     * - There are no devices.
     * - Or have already retrieved all of them.
     */
    bool search(uint8_t *newAddr, bool search_mode = true);
#endif
/**
 * @brief This seccion of code will be active if ONEWIRE_CRC = 1.
 * Each DS18B20 contains a unique 64–bit code stored in ROM.
 * The least significant 8 bits of the ROM code contain 
 * the DS18B20’s 1-Wire family code: 28h. 
 * The next 48 bits contain a unique serial number. 
 * The most significant 8 bits contain a 
 * cyclic redundancy check (CRC) byte that is calculated 
 * from the first 56 bits of the ROM code.
 */
#if ONEWIRE_CRC
    /**
     * @brief Compute a Dallas Semiconductor 8 bit CRC, these are 
     * used in the ROM and scratchpad registers.
     *
     * @param addr This pointer has the unique ID of the device.
     * @param len Is the length of the data stores in addr.
     * @return uint8_t 
     */
    static uint8_t crc8(const uint8_t *addr, uint8_t len);

#if ONEWIRE_CRC16
    /**
     * @brief Compute the 1-Wire CRC16 and compare it against the received CRC.
     * Example usage (reading a DS2408):
     *    // Put everything in a buffer so we can compute the CRC easily.
     *    uint8_t buf[13];
     *    buf[0] = 0xF0;    // Read PIO Registers
     *    buf[1] = 0x88;    // LSB address
     *    buf[2] = 0x00;    // MSB address
     *    WriteBytes(net, buf, 3);    // Write 3 cmd bytes
     *    ReadBytes(net, buf+3, 10);  // Read 6 data bytes, 2 0xFF, 2 CRC16
     *    if (!CheckCRC16(buf, 11, &buf[11])) {
     *        // Handle error.
     *    }     
     * 
     * 
     * @param input - Array of bytes to checksum.
     * @param len - How many bytes to use.
     * @param inverted_crc - The two CRC16 bytes in the received data.
     *                       This should just point into the received data,
     *                       *not* at a 16-bit integer.
     * @param crc - The crc starting value (optional).
     * @return true - CRC matches.
     * @return false - CRC NOT matches.
     */
    static bool check_crc16(const uint8_t* input, uint16_t len, const uint8_t* inverted_crc, uint16_t crc = 0);

    /**
     * @brief Compute a Dallas Semiconductor 16 bit CRC.  
     * This is required to check the integrity of data received from 
     * many 1-Wire devices.  
     * Note that the CRC computed here is *not* what you'll get from 
     * the 1-Wire network, for two reasons:
     *   1) The CRC is transmitted bitwise inverted.
     *   2) Depending on the endian-ness of your processor, the binary
     *      representation of the two-byte return value may have a different
     *      byte order than the two bytes you get from 1-Wire.
     * 
     * @param input - Array of bytes to checksum. 
     * @param len - How many bytes to use. 
     * @param crc - The crc starting value (optional). 
     * @return uint16_t - The CRC16, as defined by Dallas Semiconductor.
     */
    static uint16_t crc16(const uint8_t* input, uint16_t len, uint16_t crc = 0);
#endif
#endif
};

// Prevent this name from leaking into Arduino sketches
#ifdef IO_REG_TYPE
#undef IO_REG_TYPE
#endif

#endif // __cplusplus
#endif // OneWire_h
