#include <Wire.h>
#include <Adafruit_VL53L0X.h>

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Serial.println("VL53L0X during startup...");

  if (!lox.begin()) {
    Serial.println("VL53L0X not found");
    while (1);
  }

  Serial.println("VL53L0X initialization complete");
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false); // 測距開始(第2引数trueでデバッグ表示)

  if (measure.RangeStatus != 4) {
    Serial.print("距離: ");
    Serial.print(measure.RangeMilliMeter);
    Serial.println(" mm");
  } else {
    Serial.println("ERROR: 測定範囲外");
  }

  delay(500);
}
