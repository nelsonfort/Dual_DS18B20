#ifndef OneWire_h
#define OneWire_h

#ifdef __cplusplus

#include <stdint.h>
#include <Arduino.h>     
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
 */
class OneWire
{
  private:
    microsDelay delayMicros; /*non blocking delay in us class*/
	uint8_t pin_oneWire;

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
     *   - "pulseNotDetected"
     *   - "pulseDetected"
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
     *   - "functionFinishes"
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
     *   - "functionFinishes".
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
     *   - "functionFinishes"
     */
    enum_oneWireState write_bytes(const uint8_t *buf, uint16_t count, bool power = 0);

    /**
     * @brief Reads a byte of data from the sensor.
     * 
     * @param readBuffer The register where will be stored the data received.
     * @return enum_oneWireState 
     * - Returns "functionBusy" if the function is still working.
     * - If the function has finished, it returns:
     *   - "functionFinishes"
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
     *   - "functionFinishes".
     */
    enum_oneWireState read_bytes(uint8_t *buf, uint16_t count);

    /**
     * @brief Write one bit to the sensor.
     * NOTE THAT : The bus is always left powered at the end, see
     * note in write() about that.
     * 
     * @param v Bit to be written in the device
     * @return enum_oneWireState 
     * - Returns "functionBusy" if the function is still working.
     * - If the function has finished, it returns:
     *   - "functionFinishes"
     */
    void write_bit(uint8_t v);

    // Read a bit.
    /**
     * @brief Read a bit from the device
     * 
     * @return enum_oneWireState 
     * - Returns "functionBusy" if the function is still working.
     * - If the function has finished, it returns:
     *   - "bitReadHigh" The data that has been read is 1.
	 *   - "bitReadLow" The data that has been read is 0.
     */
    enum_oneWireState read_bit(void);
};


#endif // __cplusplus
#endif // OneWire_h
