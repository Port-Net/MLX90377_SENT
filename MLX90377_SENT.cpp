//
//    FILE: MLX90377_SENT.cpp
//  AUTHOR:  Chrisrian Port
// VERSION: 0.3.2
// PURPOSE: Arduino library for MLX90377 magnetic rotation encoder IC
//    DATE: 2025-03-12
//     URL: https://github.com/Port-Net/MLX90377_SENT


#include "MLX90377_SENT.h"


MLX90377_SENT::MLX90377_SENT(gpio_num_t pin, uint8_t tick_time_us) : RMT_SENT_RECEIVER(pin, tick_time_us) {
}

MLX90377_SENT::MLX90377_SENT(uint8_t pin, uint8_t tick_time_us) : RMT_SENT_RECEIVER((gpio_num_t)pin, tick_time_us) {
}


bool MLX90377_SENT::begin(uint16_t ticks_per_rev, int32_t clip_min, int32_t clip_max) {
  _ticks_per_rev = ticks_per_rev;
  _clip_min = clip_min;
  _clip_max = clip_max;
  return RMT_SENT_RECEIVER::begin();
}

void MLX90377_SENT::setTicksPerRev(uint16_t ticks) {
  _ticks_per_rev = ticks;
}

uint16_t MLX90377_SENT::getTicksPerRev() {
  return _ticks_per_rev;
}

void MLX90377_SENT::setDirection(int8_t direction) {
  _direction = direction;
}

int8_t MLX90377_SENT::getDirection() {
  return _direction;
}

void MLX90377_SENT::setClipRange(int32_t min, int32_t max) {
  _clip_min = min;
  _clip_max = max;
}

uint16_t MLX90377_SENT::getRawAngle() {
  return _raw_angle;
}

float MLX90377_SENT::getAngle(Angle_Mode_t type) {
  float value;
  switch(type) {
    case A_MODE_RADIANS:
      value = 2.0 * M_PI * _angle / _ticks_per_rev;
      break;
    case A_MODE_DEGREES:
      value = 360.0 * _angle / _ticks_per_rev;
      break;
    case A_MODE_RAW:
      value = _angle;
      break;
    case A_MODE_SENSOR:
      value = _raw_angle;
      break;
    default:
      value = nanf("");
  }
  return value;
}

void MLX90377_SENT::setAngle(float angle, Angle_Mode_t mode) {
  int32_t value;
  switch(mode) {
    case A_MODE_RADIANS:
      value = 2.0 * M_PI / angle * _ticks_per_rev;
      break;
    case A_MODE_DEGREES:
      value = 360.0 / angle * _ticks_per_rev;
      break;
    case A_MODE_RAW:
      value = angle;
    default:
      return;
  }
  _angle = value;
}

float MLX90377_SENT::getAngularSpeed(Speed_Mode_t mode) {
  float value;
  switch(mode) {
    case S_MODE_DEGREES:
      value = _speed / _ticks_per_rev * 360;
      break;
    case S_MODE_RADIANS:
      value = _speed / _ticks_per_rev * 2 * M_PI;
      break;
    case S_MODE_RPM:
      value = _speed / _ticks_per_rev / 60;
      break;
    case S_MODE_RAW:
      value = _speed;
      break;
    default:
      value = nanf("");
  }
  return value;
}

int32_t MLX90377_SENT::getRevolutions() {
  return _angle % _ticks_per_rev;
}

void MLX90377_SENT::setRevolutions(int32_t revolutions) {
  _angle %= _ticks_per_rev;
  _angle += revolutions * _ticks_per_rev;
}

uint32_t MLX90377_SENT::getLastMeasurementTs() {
  return _last_measurement_ts;
}

uint16_t MLX90377_SENT::getSerialStatus() {
  return _serial_status;
}

uint32_t MLX90377_SENT::getLastSerialStatusTs() {
  return _last_serial_status_ts;
}

float MLX90377_SENT::getTemperature() {
  return _temperature;
}

uint32_t MLX90377_SENT::getLastTemperatureTs() {
  return _last_temperature_ts;
}

bool MLX90377_SENT::processData(uint32_t timestamp) {
  uint32_t received_us = timestamp;
  uint8_t ct = _nibbles[3] << 4 | _nibbles[4];
  _last_counter++;
  if(!_packet_count) {
    _last_counter = ct;
  }
  if(ct != _last_counter) {
    _missed_packets += ct - _last_counter;
    _last_counter = ct;
    //Serial.printf("missed %d\r\n", ct);
  }
  if((~(_nibbles[0]) & 0x0f) != _nibbles[5]) {
    //Serial.printf("wrong invert %d\r\n", ct);
    return false;
  }
  _raw_angle = _nibbles[0] << 8 | _nibbles[1] << 4 | _nibbles[2];
  if(_last_raw_angle != 0x7FFF) {
    int16_t delta_ticks = _raw_angle - _last_raw_angle;
    uint32_t delta_t = received_us - _last_measurement_us;
    if(delta_ticks > 2048) {
      delta_ticks -= 4096;
    } else if(delta_ticks < -2048) {
      delta_ticks += 4096;
    }
    _angle += delta_ticks * _direction;
    if(_angle < _clip_min) {
      _angle += (_clip_max - _clip_min + 1);
    } else if(_angle > _clip_max) {
      _angle -= (_clip_max - _clip_min + 1);
    }
    _speed = 10000000.0 * delta_ticks / delta_t * _direction; 
  }
  _last_raw_angle = _raw_angle;
  _last_measurement_ts = millis();
  _last_measurement_us = received_us;
  return true;
}

bool MLX90377_SENT::processSerial(uint8_t msg_id, uint16_t msg_data) {
  switch(msg_id) {
    case 1:
      _serial_status = msg_data;
      _last_serial_status_ts = millis();
      break;
    case 0x23:
      _temperature = msg_data / 8 - 73; // seems to be only in 1° steps
      _last_temperature_ts = millis();
      break;
  }
  return true;
}

