# PMS Library
Arduino library for Plantower PMS sensors.
Supports PMS x003 sensors (1003, 3003, 5003(S/T/ST), 6003, 7003, A003).
## Installation
Just use Arduino Library Manager and search "PMS Library" in Sensors category.
## Main assumptions
- easy as possible,
- minimal memory consumption,
- non-blocking functions,
- supporting a wide range of PMS sensors from Plantower,
- supporting additional modes e.g.: sleeping, passive, active (depends on the sensor model).

As a data source you can use **any object** that implements the **Stream class**, such as Wire, Serial, EthernetClient, e.t.c.
## Basic example
Read in active mode.
> Default mode is active after power up. In this mode sensor would send serial data to the host automatically. The active mode is divided into two sub-modes: stable mode and fast mode. If the concentration change is small the sensor would run at stable mode with the real interval of 2.3s. And if the change is big the sensor would be changed to fast mode automatically with the interval of 200~800ms, the higher of the concentration, the shorter of the interval.
```cpp
#include "PMS.h"

PMS pms(&Serial1);
PMS_CONCENTRATION data;

void setup() {
  Serial.begin(9600);
}

void loop() {
  if (pms.read(&data)) {
    Serial.print("PM  1.0 (ug/m3): ");
    Serial.println(data.pm10_standard);

    Serial.print("PM  2.5 (ug/m3): ");
    Serial.println(data.pm25_standard);

    Serial.print("PM 10.0 (ug/m3): ");
    Serial.println(data.pm100_standard);
	
    Serial.print("PM  1.0 (ug/m3)AE: ");
    Serial.println(data.pm10_env);

    Serial.print("PM  2.5 (ug/m3)AE: ");
    Serial.println(data.pm25_env);

    Serial.print("PM 10.0 (ug/m3)AE: ");
    Serial.println(data.pm100_env);
	
    Serial.print(" 0.3um Particle Count: ");
    Serial.println(data.particles_03um);
	
    Serial.print(" 0.5um Particle Count: ");
    Serial.println(data.particles_05um);
	
    Serial.print(" 1.0um Particle Count: ");
    Serial.println(data.particles_10um);
	
    Serial.print(" 2.5um Particle Count: ");
    Serial.println(data.particles_25um);
	
    Serial.print(" 5.0um Particle Count: ");
    Serial.println(data.particles_50um);
	
    Serial.print("10.0um Particle Count: ");
    Serial.println(data.particles_100um);
	
    Serial.println(data.firmware_version);
    Serial.println(data.error_code);
    Serial.println();
  }
  // Do other stuff...
}
```
## Output
```
PM  1.0 (ug/m3): 13
PM  2.5 (ug/m3): 18
PM 10.0 (ug/m3): 23

PM  1.0 (ug/m3): 12
PM  2.5 (ug/m3): 19
PM 10.0 (ug/m3): 24

...
```
## Advanced example
Read in passive mode but not the best way (see additional remarks). 
```cpp
#include "PMS.h"

PMS pms(&Serial1);
PMS_CONCENTRATION data;

void setup() {
  Serial.begin(9600);
  pms.passiveMode();    // Switch to passive mode
}

void loop() {
  Serial.println("Waking up, wait 30 seconds for stable readings...");
  pms.wakeUp();
  delay(30000);

  Serial.println("Send read request...");
  pms.requestRead();

  Serial.println("Wait max. 1 second for read...");
  if (pms.readUntil(&data)) {
    Serial.print("PM  1.0 (ug/m3): ");
    Serial.println(data.pm10_standard);

    Serial.print("PM  2.5 (ug/m3): ");
    Serial.println(data.pm25_standard);

    Serial.print("PM 10.0 (ug/m3): ");
    Serial.println(data.pm100_standard);
  } else {
    Serial.println("No data.");
  }

  Serial.println("Going to sleep for 60 seconds.");
  pms.sleep();
  delay(60000);
}
```
## Output
```
Waking up, wait 30 seconds for stable readings...
Send read request...
Reading data...
PM  1.0 (ug/m3): 13
PM  2.5 (ug/m3): 18
PM 10.0 (ug/m3): 23
Going to sleep for 60 seconds.
Waking up, wait 30 seconds for stable readings...
Send read request...
Reading data...
PM  1.0 (ug/m3): 12
PM  2.5 (ug/m3): 19
PM 10.0 (ug/m3): 24
Going to sleep for 60 seconds.

...
```
## Additional remarks
Tested with PMS 7003 and ESP-12E Development Board.
All Plantower PMS sensors uses the same protocol (let me know if you have any problems).

Please consider, that delay() function in examples is a blocking function.  
Try to avoid such a solution if your project requires it (see Expert.ino example in examples directory).

For more accurate measurements, you can read several samples (in passive or active mode) and calculate the average.
> Stable data should be got at least 30 seconds after the sensor wakeup from the sleep mode because of the fan's performance.
