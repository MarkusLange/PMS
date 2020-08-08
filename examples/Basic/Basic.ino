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