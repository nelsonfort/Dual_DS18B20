#include <ds18b20.h>
#include <microsDelay.h>
#include <OneWire.h>


ds18b20 sensor1(22);
float sensor1_measure = 0;

void setup() {
  Serial.begin(115200);
  pinMode(21, OUTPUT);
  pinMode(19, OUTPUT);
  pinMode(18, OUTPUT); 
}

void loop() 
{
  delay(100);
  if( sensor1.takeMeasure(&sensor1_measure) == functionFinishes )
  {
    Serial.print("El valor de temperatura del sensor1 es: ");
    Serial.print(sensor1_measure, 10);
    Serial.print("\n");
    delay(2000);
  }

}
