#include <Wire.h>
#include <Adafruit_VL53L0X.h>

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup() {
  Serial.begin(9600);
  Wire.begin();

  if (!lox.begin()) {
    Serial.println("VL53L0X センサが見つかりません。配線を確認してください。");
    while (1);
  }

  Serial.println("VL53L0X 初期化成功");
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false); // trueでデバッグ出力

  if (measure.RangeStatus != 4) {  // VL53L0X_ERROR_NONE
    Serial.print("距離: ");
    Serial.print(measure.RangeMilliMeter);
    Serial.println(" mm");
  } else {
    Serial.println("測定エラー");
  }

  delay(500);
}
