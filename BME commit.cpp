
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BME280.h>
/*
3.3V a VCC y SDO
SCL A5
SDA A4
*/ 


Adafruit_BME280 bme; // use I2C interface
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();

void setup() {
Serial.begin(9600);


if (!bme.begin()) {
 Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
 while (1) delay(10);
}


}

void loop() {
sensors_event_t temp_event, pressure_event, humidity_event;
bme_temp->getEvent(&temp_event);
bme_pressure->getEvent(&pressure_event);
bme_humidity->getEvent(&humidity_event);

Serial.print(F("Temperature = "));      //No quitar la F, ahorra RAM
Serial.print(temp_event.temperature);
Serial.println(" *C");

Serial.print(F("Humidity = "));
Serial.print(humidity_event.relative_humidity);
Serial.println(" %");

Serial.print(F("Pressure = "));
Serial.print(pressure_event.pressure);
Serial.println(" hPa");

Serial.println();
delay(1000);
}


