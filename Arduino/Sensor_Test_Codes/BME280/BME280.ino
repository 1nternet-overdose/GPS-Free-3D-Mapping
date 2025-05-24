#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

Adafruit_BME280 bme;

void setup() {
  Serial.begin(9600);
  Serial.println("BME280 during startup...");

  if (!bme.begin(0x76)) {
    Serial.println("BME280 not found");
    while (1);
  }

  Serial.println("BME280 initialization complete");
}

void loop() {
  float temp = bme.readTemperature();           // 温度(摂氏)
  float pressure = bme.readPressure() / 100.0F; // 気圧(hPa単位へ換算)
  float humidity = bme.readHumidity();          // 湿度(%)
  float altitude = bme.readAltitude(1013.25);   // 高度(気圧から計算，基準気圧は気象庁発表値を参考)

  Serial.print("気温: "); Serial.print(temp);     Serial.println(" ℃");
  Serial.print("湿度: "); Serial.print(humidity); Serial.println(" %");
  Serial.print("気圧: "); Serial.print(pressure); Serial.println(" hPa");
  Serial.print("高度: "); Serial.print(altitude); Serial.println(" m");

  delay(1000);
}
