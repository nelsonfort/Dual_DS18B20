#include <ds18b20.h>
#include <microsDelay.h>
#include <OneWire.h>

#define SELECT_A_Pin 25
#define SELECT_B_Pin 26
#define DS18B20_DATA_Pin 27

/*Those variables are necesary to work with the library*/
ds18b20 sensors(DS18B20_DATA_Pin); /*initiates the oneWire on a pin*/
uint8_t sensor_x;
float sensor_A_measure = 0; /*uses a float value to store the measure*/
float sensor_B_measure = 0; /*uses a float value to store the measure*/

void setup() {
  Serial.begin(115200);
  pinMode(SELECT_A_Pin,OUTPUT);
  pinMode(SELECT_B_Pin,OUTPUT);
  delay(200);
  sensor_x = 1;
  digitalWrite(SELECT_A_Pin,HIGH);
  digitalWrite(SELECT_B_Pin,LOW);
}

void loop() 
{
  delay(10);
  Serial.print(".");
  if( (sensor_x == 1) && (sensors.takeMeasure(&sensor_A_measure) == functionFinishes ))
  {  
    sensor_x = 2;
    digitalWrite(SELECT_A_Pin,LOW);
    digitalWrite(SELECT_B_Pin,HIGH);
    Serial.print("\nEl valor de temperatura del sensor A es: ");
    Serial.print(sensor_A_measure, 10);
    Serial.print("\n");
    delay(2000);
  }
  if( (sensor_x == 2) && (sensors.takeMeasure(&sensor_B_measure) == functionFinishes ))
  {  
    sensor_x = 1;
    digitalWrite(SELECT_A_Pin,HIGH);
    digitalWrite(SELECT_B_Pin,LOW);
    Serial.print("\nEl valor de temperatura del sensor B es: ");
    Serial.print(sensor_B_measure, 10);
    Serial.print("\n");
    delay(2000);
  }
 
}
