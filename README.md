# Dual_DS18B20
This repository is made to create a structure to read two different sensors and detect which sensor has a given address to reference it without assumptions.

The library has the capabilities to let the microcontroller do other tasks while the OneWire connection is performed with the two DS18B20 sensors.
This can be made because the original library was changed to not use blocking delays and in each access to a method the method returns its state to detect if it continues processing a query or the data ready to be used.

In the following image there is a first signal that send a reset condition to get a response of the sensor after the reset pulse (signal in low state for 480us). There is another signal that switches an output pin al the time to check that the delays was mitigated satisfactorily.

![Library without non blocking delay](/RDM_IMAGES/OWCTest.jpeg)

In the next image there can be the the previous image zoomed. It can be perceived some periods of time where the toogle pin is not performed because in these period of time the library is making some calculations that need some time to be completed.

![Library without non blocking delay1](/RDM_IMAGES/OWCTestZoomed1.jpeg)

The image below is more amplified to see that the pulses made are around 125ns. While the library is performing the connection to the sensor.

![Library without non blocking delay2](/RDM_IMAGES/OWCTestZoomed2.jpeg)
