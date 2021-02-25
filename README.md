# Dual_DS18B20
## **Description**
**---------------------------------------------------------------------------------------
This repository is made to create a structure to read two different sensors and detect which sensor has a given address to reference it without assumptions.
-----------------------------------------------------------------------------------------**

The library has the capabilities to let the microcontroller do other tasks while the OneWire connection is performed with the two DS18B20 sensors.
This can be made because the original library was changed to not use blocking delays and in each access to a method the method returns its state to detect if it continues processing a query or the data ready to be used.

In the following image there is a first signal that send a reset condition to get a response of the sensor after the reset pulse (signal in low state for 480us). There is another signal that switches an output pin al the time to check that the delays was mitigated satisfactorily.

![Library without non blocking delay](/RDM_IMAGES/OWCTest.jpg)

In the next image there can be the the previous image zoomed. It can be perceived some periods of time where the toogle pin is not performed because in these period of time the library is making some calculations that need some time to be completed.

![Library without non blocking delay1](/RDM_IMAGES/OWCTestZoomed1.jpg)

The image below is more amplified to see that the pulses made are around 125ns. While the library is performing the connection to the sensor.

![Library without non blocking delay2](/RDM_IMAGES/OWCTestZoomed2.jpeg)

#### Using the feature to read a sensor without knowing it's address


The image displayed below shows the procedure to read all the registers (9) that be in the sensor without using it's address to reference it. 

![ReadingAllRegs1](/RDM_IMAGES/ReadingAllRegs.jpeg)

In the following images the toogle pin is working while the library is communicating with the DS18B20 to receive the data stored on its registers.  

So the microcontroller can do other tasks during the communication.   


![ReadingAllRegs2](/RDM_IMAGES/ReadingAllRegsZoom1.jpeg)


![ReadingAllRegs3](/RDM_IMAGES/ReadingAllRegsZoom2.jpeg)

*-------------------------25/02----------------------------------*
Updated the library removing direct_write and direct_read methods that
not works as direct as expected. digitalRead and digitalWrite are much faster.

There is a new library ds18b20 that uses the OneWire customized library.
The ds18b20 is used to initialize the sensor automatically and read the 
temperature with only a few of methods.

These libraries are tested with large delays and the results obtained are correct. So this implementation can be used with other code without problems.

**Note:** to implement this behavior it was required to implement a small blocking delay for each access to a method inside the OneWire library. This blocking delay is lower than 70us in all of the cases.

The following image shows the two sensors working. In this case there is some delays to make sure the library is working properly.
In the serial monitor it can be seen both sensors working and its temperature is shown.
![two_sensors_working](/RDM_IMAGES/two_sensors_working.png)

This are the waveforms of the data transmission to the sensors. 
![waveforms_2_sensors](/RDM_IMAGES/waveforms_2_sensors.jpg)

### Library installation
Download the library and add it to the Arduino library.

### Connection
Select 2 data pins in the sketch that will be use to connect the two sensor. Its better use two data pins to avoid the use of transistors to select from one sensor to another.  
The sensors are active powered so the ground pin is connected to GND and
the power pin is connected to the 3V3 pin.  
Data pin is connected with a pullup resistor of 4.7K.  

![connection1](/RDM_IMAGES/connection1.jpg)  

![connection2](/RDM_IMAGES/connection2.jpg)  

This library is based in the [OneWire Arduino Library](https://www.arduinolibraries.info/libraries/one-wire) 