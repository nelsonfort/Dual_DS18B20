/*DS18B20 temperature sensor library.
 * - This library is a non-blocking library.
 * - MAX BLOCKING TIME PER FUNCTION CALL: 70us 
 * - This library is robust to blocking delays between functions calls. MAX BLOCKING DELAY BETWEEN CALLS: unlimited
 * - IMPORTANT: This library only supports 1 sensor connected to each pin simultaneously.
 */
#include <Arduino.h>
#include <OneWire.h>

class ds18b20
{
	public:
		ds18b20( uint8_t pin ){ begin(pin); };
		
		void begin( uint8_t pin );
		
		/*returns "functionBusy" or "functionFinishes". When return functionFinishes, readBuffer was loaded with data*/
		enum_oneWireState takeMeasure(float *readBuffer); 
	
	
	
	private:
		OneWire sensor;
		uint8_t pin_oneWire;
		
		enum_oneWireState convert_T( void ); /* sends the convert T command to the sensor for the initialization of the measure. Returns "functionBusy" or "functionFinishes" */
		
		enum_oneWireState read_Scratchpad(void); /*sends the command to read the internal registers of the sensor. Returns "functionBusy" or "functionFinishes"*/
		
		enum_oneWireState waitConversionReady( void ); /* checks if the sensor has finished the conversion. Returns "functionBusy" or "functionFinishes */
		
		enum_oneWireState readTemperature( float *readBuffer ); /*returns "functionBusy" or "functionFinishes". When returns functionFinishes, readBuffer was loaded with data*/
		
		float receivedData_to_float( uint8_t *bytes, uint8_t byteCount ); /*converts the received data to Celcius on a float*/
};