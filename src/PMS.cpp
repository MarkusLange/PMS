#include "Arduino.h"
#include "PMS.h"

PMS::PMS(HardwareSerial* stream) {
  (*stream).begin(BAUD_RATE);
  this->_stream = stream;
}

// Standby mode. For low power consumption and prolong the life of the sensor.
void PMS::sleep() {
  uint8_t command[] = { 0x42, 0x4D, 0xE4, 0x00, 0x00, 0x01, 0x73 };
  _stream->write(command, sizeof(command));
}

// Operating mode. Stable data should be got at least 30 seconds after the sensor wakeup from the sleep mode because of the fan's performance.
void PMS::wakeUp() {
  uint8_t command[] = { 0x42, 0x4D, 0xE4, 0x00, 0x01, 0x01, 0x74 };
  _stream->write(command, sizeof(command));
}

// Active mode. Default mode after power up. In this mode sensor would send serial data to the host automatically.
void PMS::activeMode() {
  uint8_t command[] = { 0x42, 0x4D, 0xE1, 0x00, 0x01, 0x01, 0x71 };
  _stream->write(command, sizeof(command));
  _mode = MODE_ACTIVE;
}

// Passive mode. In this mode sensor would send serial data to the host only for request.
void PMS::passiveMode() {
  uint8_t command[] = { 0x42, 0x4D, 0xE1, 0x00, 0x00, 0x01, 0x70 };
  _stream->write(command, sizeof(command));
  _mode = MODE_PASSIVE;
}

// Request read in Passive Mode.
void PMS::requestRead() {
  if (_mode == MODE_PASSIVE) {
    uint8_t command[] = { 0x42, 0x4D, 0xE2, 0x00, 0x00, 0x01, 0x71 };
    _stream->write(command, sizeof(command));
  }
}

// Non-blocking function for parse response.
bool PMS::read(PMS_CONCENTRATION *_data) {
  loop(_data);
  
  return _status == STATUS_OK;
}

// Blocking function for parse response. Default timeout is 1s.
bool PMS::readUntil(PMS_CONCENTRATION *_data, uint16_t timeout) {
  uint32_t start = millis();
  do {
    loop(_data);
    if (_status == STATUS_OK) break;
  } while (millis() - start < timeout);
  
  return _status == STATUS_OK;
}

void PMS::loop(PMS_CONCENTRATION *_data) {
  _status = STATUS_WAITING;
  if (_stream->available()) {
    //uint8_t data = _stream->read();
    
    switch (_index) {
    case 0:
      if (_stream->peek() != 0x42) {
		_stream->read();
        return;
      }
      _header[0] = _stream->read();
	  _index = 1;
      break;
    
    case 1:
      if (_stream->peek() != 0x4D) {
		_stream->read();
        _index = 0;
        return;
      }
      _header[1] = _stream->read();
	  _index = 2;
      break;
    
    case 2:
      _header[2] = _stream->read();
      _index = 3;
      break;
	
	case 3:
	  _header[3] = _stream->read();
      _frameLen = makeWord(_header[2], _header[3]);
	  // Unsupported sensor, different frame length, transmission error e.t.c.
      if (_frameLen != 2 * 9 + 2 && _frameLen != 2 * 13 + 2 && _frameLen != 2 * 17 + 2) {
        _index = 0;
        return;
      }
	  _index = 4;
	  break;
	
	case 4:
	  _stream->readBytes(_frame, _frameLen);
	  
      // get checksum ready
	  _calculatedChecksum = 0;
      for (uint8_t i = 0; i < 4; i++)
        _calculatedChecksum += _header[i];
      for (uint8_t i = 0; i < _frameLen - 2; i++)
        _calculatedChecksum += _frame[i];
	  
	  _checksum = makeWord(_frame[_frameLen - 2], _frame[_frameLen - 1]);
	  
      if (_calculatedChecksum == _checksum) { // PMS3003 only common to all
	_data->framelength        = _frameLen;
        // Standard Particles, CF=1.
        _data->pm10_standard      = makeWord(_frame[ 0], _frame[ 1]);
        _data->pm25_standard      = makeWord(_frame[ 2], _frame[ 3]);
        _data->pm100_standard     = makeWord(_frame[ 4], _frame[ 5]);
        
        // Atmospheric Environment.
        _data->pm10_env           = makeWord(_frame[ 6], _frame[ 7]);
        _data->pm25_env           = makeWord(_frame[ 8], _frame[ 9]);
        _data->pm100_env          = makeWord(_frame[10], _frame[11]);
		
		if (_frameLen == 2 * 13 + 2 || _frameLen == 2 * 17 + 2) { // PMS1003 PMS5003S PMS5003T PMS6003 PMS7003 PMSA003
		  // Particle Count
		  _data->particles_03um   = makeWord(_frame[12], _frame[13]);
		  _data->particles_05um   = makeWord(_frame[14], _frame[15]);
		  _data->particles_10um   = makeWord(_frame[16], _frame[17]);
		  _data->particles_25um   = makeWord(_frame[18], _frame[19]);
		  // PMS1003 PMS5003S PMS6003 PMS7003 PMSA003
		  _data->particles_50um   = makeWord(_frame[20], _frame[21]);
		  _data->particles_100um  = makeWord(_frame[22], _frame[23]);
		  
		  // PMS5003T
		  _data->temperature      = makeWord(_frame[20], _frame[21]) / 10;
		  _data->humidity         = makeWord(_frame[22], _frame[23]) / 10;
		}
		
		if (_frameLen == 2 * 13 + 2) {
	          // PMS7003 PMSA003
		  _data->firmware_version = _frame[24];
		  _data->error_code       = _frame[25];
		  
		  // PMS5003S
		  _data->formaldehyde     = makeWord(_frame[24], _frame[25]) / 1000;
		}
		
		if (_frameLen == 2 * 17 + 2) { //PMS5003ST
		  // Environment
		  _data->formaldehyde     = makeWord(_frame[24], _frame[25]) / 1000;
		  _data->temperature      = makeWord(_frame[26], _frame[27]) / 10;
		  _data->humidity         = makeWord(_frame[28], _frame[29]) / 10;
		  
		  _data->firmware_version = _frame[32];
		  _data->error_code       = _frame[33];
		}
		
		_index = 0;
		_status = STATUS_OK;
		return;
      }
	  else {
	    _index = 0;
		//return;
	  }
	  break;
	}
  }
}

/*
void PMS::loop(PMS_CONCENTRATION *_data) {
  _status = STATUS_WAITING;
  if (_stream->available()) {
    uint8_t ch = _stream->read();

    switch (_index) {
    case 0:
      if (ch != 0x42) {
        return;
      }
      _calculatedChecksum = ch;
      break;

    case 1:
      if (ch != 0x4D) {
        _index = 0;
        return;
      }
      _calculatedChecksum += ch;
      break;

    case 2:
      _calculatedChecksum += ch;
      _frameLen = ch << 8;
      break;

    case 3:
      _frameLen |= ch;
      // Unsupported sensor, different frame length, transmission error e.t.c.
      if (_frameLen != 2 * 9 + 2 && _frameLen != 2 * 13 + 2) {
        _index = 0;
        return;
      }
      _calculatedChecksum += ch;
      break;

    default:
      if (_index == _frameLen + 2) {
        _checksum = ch << 8;
      }
      else if (_index == _frameLen + 2 + 1) {
        _checksum |= ch;

        if (_calculatedChecksum == _checksum) {
          _status = STATUS_OK;

          // Standard Particles, CF=1.
          _data->PM_SP_UG_1_0 = makeWord(_payload[0], _payload[1]);
          _data->PM_SP_UG_2_5 = makeWord(_payload[2], _payload[3]);
          _data->PM_SP_UG_10_0 = makeWord(_payload[4], _payload[5]);

          // Atmospheric Environment.
          _data->PM_AE_UG_1_0 = makeWord(_payload[6], _payload[7]);
          _data->PM_AE_UG_2_5 = makeWord(_payload[8], _payload[9]);
          _data->PM_AE_UG_10_0 = makeWord(_payload[10], _payload[11]);
        }

        _index = 0;
        return;
      }
      else {
        _calculatedChecksum += ch;
        uint8_t payloadIndex = _index - 4;

        // Payload is common to all sensors (first 2x6 bytes).
        if (payloadIndex < sizeof(_payload)) {
          _payload[payloadIndex] = ch;
        }
      }
      break;
    }

    _index++;
  }
}
*/
