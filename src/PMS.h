#ifndef PMS_H
#define PMS_H

#include "Stream.h"

typedef struct {
  uint16_t framelen;        ///< How long this data chunk is
  uint16_t pm10_standard,   ///< Standard PM1.0
      pm25_standard,        ///< Standard PM2.5
      pm100_standard;       ///< Standard PM10.0
  uint16_t pm10_env,        ///< Environmental PM1.0
      pm25_env,             ///< Environmental PM2.5
      pm100_env;            ///< Environmental PM10.0
  uint16_t particles_03um,  ///< 0.3um Particle Count
      particles_05um,       ///< 0.5um Particle Count
      particles_10um,       ///< 1.0um Particle Count
      particles_25um,       ///< 2.5um Particle Count
      particles_50um,       ///< 5.0um Particle Count
      particles_100um;      ///< 10.0um Particle Count
  uint8_t firmware_version, ///< Firmware Version
      error_code;           ///< Device error code
  float temperature,        ///< Temperature °C
      humidity,             ///< Humidity %
      formaldehyde;         ///< Formaldehyde mg/m3
  uint16_t unused;          ///< Unused
  uint16_t checksum;        ///< Packet checksum
} PMS_CONCENTRATION;

class PMS
{
public:
  static const uint16_t SINGLE_RESPONSE_TIME = 1000;
  static const uint16_t TOTAL_RESPONSE_TIME  = 1000 * 10;
  static const uint16_t STEADY_RESPONSE_TIME = 1000 * 30;

  static const uint16_t BAUD_RATE = 9600;

  PMS(HardwareSerial* stream);
  void sleep();
  void wakeUp();
  void activeMode();
  void passiveMode();

  void requestRead();
  bool read(PMS_CONCENTRATION *data);
  bool readUntil(PMS_CONCENTRATION *data, uint16_t timeout = SINGLE_RESPONSE_TIME);

private:
  enum STATUS { STATUS_WAITING, STATUS_OK };
  enum MODE { MODE_ACTIVE, MODE_PASSIVE };

  uint8_t _payload[12];
  uint8_t _header[4];
  uint8_t _frame[36];
  Stream* _stream;
  STATUS _status;
  MODE _mode = MODE_ACTIVE;

  uint8_t _index = 0;
  uint16_t _frameLen;
  uint16_t _checksum;
  uint16_t _calculatedChecksum;

  void loop(PMS_CONCENTRATION *data);
};

#endif