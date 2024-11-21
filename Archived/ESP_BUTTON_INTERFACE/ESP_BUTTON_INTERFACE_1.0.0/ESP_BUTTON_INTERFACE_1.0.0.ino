#include "ESP_BUTTON_INTERFACE.h"

ESP_PIN_DRIVER BUTTON_DRIVER(4);

void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
Serial.println("INIT:");
}

void loop() {
  // put your main code here, to run repeatedly:
Serial.println(BUTTON_DRIVER.Run());
}
