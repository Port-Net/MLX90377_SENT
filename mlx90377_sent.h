#ifndef MLX90377_SENT_H
#define MLX90377_SENT_H


#include "Arduino.h"
#include "rmt_sent_receiver.h"

class MLX90377_SENT : public RMT_SENT_RECEIVER {
public:
  static const int8_t CW = 1;
  static const int8_t CCW = -1;
  enum Angle_Mode_t {
    A_MODE_DEGREES,
    A_MODE_RADIANS,
    A_MODE_RAW,
    A_MODE_SENSOR,
  };
  enum Speed_Mode_t {
    S_MODE_DEGREES,
    S_MODE_RADIANS,
    S_MODE_RPM,
    S_MODE_RAW,
  };
  MLX90377_SENT(gpio_num_t pin, uint8_t _tick_time_us = 3);

  bool begin(uint16_t ticks_per_rev = 4096, int32_t clip_min = INT32_MIN, int32_t clip_max = INT32_MAX);

  /**
   * @brief Set the Tick Per Rev
   *   if you have some gear in between
   *   for std turn encoder it is 4096
   * 
   * @param ticks_per_rev 
   */
  void setTicksPerRev(uint16_t ticks_per_rev);
  uint16_t getTicksPerRev();
  /**
   * @brief Set the Clipping Range
   * i.e. 0 to 4095 to get some kind of turn encoder
   *   or  -gear_ratio * 4096 to gear_ratio * 4096
   * @param min 
   * @param max 
   */
  void setClipRange(int32_t min, int32_t max);
  void setDirection(int8_t direction);
  int8_t getDirection();

  uint16_t getRawAngle();
  float getAngle(Angle_Mode_t type = A_MODE_RADIANS);
  void setAngle(float angle = 0, Angle_Mode_t type = A_MODE_RADIANS);

  uint16_t getSerialStatus();
  float getTemperature();
  uint32_t getLastMeasurementTs();
  uint32_t getLastSerialStatusTs();
  uint32_t getLastTemperatureTs();

  float getAngularSpeed(Speed_Mode_t mode = S_MODE_RADIANS);
  int32_t getRevolutions();

  /**
   * @brief Set the revolutions to n by keeping the in_turn part
   * 
   * @param revolutions 
   */
  void setRevolutions(int32_t revolutions = 0);

protected:
  bool processData() override;
  bool processSerial(uint8_t msg_id, uint16_t msg_data) override;
  int8_t  _direction = CW;
  uint32_t _last_measurement_ts = 0;
  uint32_t _last_serial_status_ts = 0;
  uint32_t _last_temperature_ts = 0;
  uint32_t _last_measurement_us = 0;
  uint16_t  _last_raw_angle  = 0;
  int32_t  _angle = 0;
  float _speed = 0;
  uint16_t _raw_angle = 0;
  uint8_t _last_counter = 0;
  uint8_t _packet_counter = 0;
  uint32_t _missed_packets = 0;
  uint16_t _serial_status = 0;
  float _temperature = 0;
  uint16_t _ticks_per_rev = 4096;
  int32_t _clip_min;
  int32_t _clip_max;
};


#endif