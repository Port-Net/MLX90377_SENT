#include <Arduino.h>
#include "mlx90377_sent.h"

MLX90377_SENT mlx90377(GPIO_NUM_38);

void setup() {
  delay(5000); //wait for monitor to connect
  Serial.begin(115200);
  Serial.println("init");

  mlx90377.begin();
  mlx90377.setClipRange(0, 4095);

  Serial.println("init done");
}

void loop() {
  static uint32_t last_print = 0;
  uint32_t ti = millis();
  if((ti - last_print) > 500) {
    last_print = ti;
    Serial.printf("state:%d angle:%d temp:%.2f°C err: %d\r\n",
        mlx90377.getStatus(), mlx90377.getAngle(MLX90377_SENT::A_MODE_SENSOR), mlx90377.getTemperature(), mlx90377.getErrorCount());
  }
}
