//
//    FILE: mlx90377_sensor.cpp
//  AUTHOR: Christian Port
// PURPOSE: demo
//     URL: https://github.com/Port-Net/MLX90377_SENT
//
//  Example for using MLX90377ADB-x3x rotation sensor based on rmt_sent receiver

#include <Arduino.h>
#include "MLX9377_SENT.h"

// create the objects using default tick time 3ms
MLX90377_SENT mlx90377(GPIO_NUM_38);
MLX90377_SENT mlx90377_2(GPIO_NUM_39);

void print_uint8_t(uint8_t x) {
  Serial.println(x);
}

void setup() {
  delay(5000); //wait for serial monitor to connect
  Serial.begin(115200);
  Serial.println("init");

  // initialize the sensor with gear ratio 1:15, clipping to one rotation
  mlx90377.begin(15 * 4096, 0, 15*4096 - 1);
  mlx90377_2.begin(15 * 4096, 0, 15*4096 - 1);
  // switch the direction of second wheel 
  mlx90377_2.setDirection(MLX90377_SENT::CCW);
  //mlx90377.begin();
  //mlx90377.setClipRange(0, 4095);

  Serial.println("init done");
}

void loop() {
  static uint32_t last_print = 0;
  uint32_t ti = millis();
  if((ti - last_print) > 500) {
    last_print = ti;
    Serial.printf("1: state:%d angle:%.2f temp:%.2f°C err:%d pkt:%d\r\n",
        mlx90377.getStatus(), mlx90377.getAngle(MLX90377_SENT::A_MODE_DEGREES), mlx90377.getTemperature(), mlx90377.getErrorCount(), mlx90377.getPacketCount());
    Serial.printf("2: state:%d angle:%.2f temp:%.2f°C err:%d pkt:%d\r\n",
        mlx90377_2.getStatus(), mlx90377_2.getAngle(MLX90377_SENT::A_MODE_DEGREES), mlx90377_2.getTemperature(), mlx90377_2.getErrorCount(), mlx90377_2.getPacketCount());
  }
}
