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