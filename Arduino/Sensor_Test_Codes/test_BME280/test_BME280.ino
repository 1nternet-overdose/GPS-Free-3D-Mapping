#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

Adafruit_BME280 bme;

void setup() {
  Serial.begin(9600);
  if (!bme.begin(0x76)) {
    Serial.println("BME280 センサが見つかりません。接続とアドレスを確認してください。");
    while (1);
  }

  Serial.println("BME280 初期化完了");
}

void loop() {
  float temp = bme.readTemperature();
  float pressure = bme.readPressure() / 100.0F; // hPa
  float humidity = bme.readHumidity();
  float altitude = bme.readAltitude(1013.25);  // 基準気圧は気象庁発表値などに合わせる

  Serial.print("気温: "); Serial.print(temp); Serial.println(" ℃");
  Serial.print("湿度: "); Serial.print(humidity); Serial.println(" %");
  Serial.print("気圧: "); Serial.print(pressure); Serial.println(" hPa");
  Serial.print("高度: "); Serial.print(altitude); Serial.println(" m");

  delay(1000);
}
