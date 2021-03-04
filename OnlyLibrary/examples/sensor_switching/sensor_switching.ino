#include <ds18b20.h>
#include <microsDelay.h>
#include <OneWire.h>

/*Those variables are necesary to work with the library*/
ds18b20 sensor(22); /*initiates the oneWire on a pin*/
uint8_t oneWire_pin = 22; /*registrate the pin where is the oneWire*/
float sensor_measure = 0; /*uses a float value to store the measure*/

void setup() {
  Serial.begin(115200);
}

void loop() 
{
  delay(100);
  if( sensor.takeMeasure(&sensor_measure) == functionFinishes )
  {  
    /*when the measure ends, switchs the pin of the oneWire */
    if( oneWire_pin == 22 )
    {
      oneWire_pin = 23;
      sensor.begin(23);
      Serial.print("El valor de temperatura del sensor A es: ");  
    }
    else
    {
      oneWire_pin = 22;
      sensor.begin(22);
      Serial.print("El valor de temperatura del sensor B es: ");
    }
    Serial.print(sensor_measure, 10);
    Serial.print("\n");
    delay(2000);
  }
 
}
