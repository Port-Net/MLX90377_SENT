//
//    FILE: MLX90377_SENT.h
//  AUTHOR:  Chrisrian Port
// VERSION: 0.3.1
// PURPOSE: Arduino library for MLX90377 magnetic rotation encoder IC
//    DATE: 2025-03-10
//     URL: https://github.com/Port-Net/MLX90377_SENT

#ifndef MLX90377_SENT_H
#define MLX90377_SENT_H


#include "Arduino.h"
#include "rmt_SENT.h"

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

  /**
   * @brief Construct a new mlx90377 sent sensor object
   * 
   * @param pin connected IO
   * @param _tick_time_us SENT tick time, defaults to 3µs
   */
  MLX90377_SENT(gpio_num_t pin, uint8_t _tick_time_us = 3);
  MLX90377_SENT(uint8_t pin, uint8_t _tick_time_us = 3);

  /**
   * @brief iitialize the sensor
   * 
   * @param ticks_per_rev can be used to apply rear ratio
   * @param clip_min in sensor ticks. Used to set a lower limit of angle, if lower go to clip_max
   * @param clip_max in sensor ticks. Used to set a upper limit of angle, if higher go to clip_min
   * @return true on success
   * @return false on error
   */
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
   * @brief Set the clipping Range
   * i.e. 0 to 4095 to get some kind of turn encoder
   *   or  -gear_ratio * 4096 to gear_ratio * 4096
   * @param min 
   * @param max 
   */
  void setClipRange(int32_t min, int32_t max);

  /**
   * @brief Set the direction the sensor is counting
   * default is CW
   * @param direction as MLX90377_SENT::CW/CCW
   */
  void setDirection(int8_t direction);
  int8_t getDirection();

  /**
   * @brief Get the raw angle of the sensor
   * 
   * @return uint16_t raw sensor value
   */
  uint16_t getRawAngle();

  /**
   * @brief Get the current angle after calculations
   * one can select the angle in different formats by using mode, default is radians
   * @param mode can be A_MODE_RADIANS, A_MODE_DEGREES, A_MODE_RAW, A_MODE_SENSOR
   * @return float angle
   */
  float getAngle(Angle_Mode_t mode = A_MODE_RADIANS);

  /**
   * @brief Set the angle 
   * 
   * @param angle value to set the angle to
   * @param mode can be A_MODE_RADIANS, A_MODE_DEGREES, A_MODE_RAW
   */
  void setAngle(float angle = 0, Angle_Mode_t mode = A_MODE_RADIANS);

  /**
   * @brief Get the Serial Status of the sensor (serial id 1)
   * usualy 0x800 in working mode
   * @return uint16_t serial status
   */
  uint16_t getSerialStatus();

  /**
   * @brief Get the temperature of the sensor (serial id 0x23)
   * in °C
   * @return float temperature
   */
  float getTemperature();

  /**
   * @brief Get the timestamp in millis the last measurement has arrived
   * 
   * @return uint32_t 
   */
  uint32_t getLastMeasurementTs();

  /**
   * @brief Get the timestamp in millis the last serial status has arrived
   * 
   * @return uint32_t 
   */
  uint32_t getLastSerialStatusTs();

  /**
   * @brief Get the timestamp in millis the last temperature has arrived
   * 
   * @return uint32_t 
   */
  uint32_t getLastTemperatureTs();

  /**
   * @brief Get the angular speed which is calculated on measurement arrival
   * one can select the speed in different formats by using mode, default is radians/s
   * @param mode can be S_MODE_RADIANS, S_MODE_DEGREES, S_MODE_RPM
   * @return float speed
   */
  float getAngularSpeed(Speed_Mode_t mode = S_MODE_RADIANS);

  /**
   * @brief Get the revolutions 
   * based on the setting on ticks per rev
   * @return int32_t 
   */
  int32_t getRevolutions();

  /**
   * @brief Set the revolutions to n by keeping the in_turn part
   * 
   * @param revolutions 
   */
  void setRevolutions(int32_t revolutions = 0);

protected:
  bool processData(uint32_t timestamp) override;
  bool processSerial(uint8_t msg_id, uint16_t msg_data) override;
  int8_t  _direction = CW;
  uint32_t _last_measurement_ts = 0;
  uint32_t _last_serial_status_ts = 0;
  uint32_t _last_temperature_ts = 0;
  uint32_t _last_measurement_us = 0;
  uint16_t  _last_raw_angle  = 0x7FFF;
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