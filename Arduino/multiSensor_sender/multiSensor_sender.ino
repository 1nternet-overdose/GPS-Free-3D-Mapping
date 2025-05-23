#include <Wire.h>
#include <MPU6050.h>
#include <QMC5883LCompass.h>
#include <VL53L0X.h>
#include <BME280I2C.h>

// センサインスタンス
MPU6050 mpu;
QMC5883LCompass compass;
VL53L0X vl53;
BME280I2C bme;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 接続失敗");
    while (1);
  }

  // QMC5883L
  compass.init();
  compass.setCalibration(-561, 1193, -1946, -200, -421, -100); // 環境に応じて変更

  // VL53L0X
  vl53.init();
  vl53.setTimeout(500);
  vl53.startContinuous();

  // BME280（I2Cアドレス自動検出）
  bool status = bme.begin();
  if (!status) {
    Serial.println("BME280 接続失敗");
    while (1);
  }

  Serial.println("全センサ初期化完了");
}

void loop() {
  // MPU6050（加速度・ジャイロ）
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // QMC5883L（地磁気）
  compass.read();
  int mx = compass.getX();
  int my = compass.getY();
  int mz = compass.getZ();

  // VL53L0X（距離）
  int distance = vl53.readRangeContinuousMillimeters();
  if (vl53.timeoutOccurred()) distance = -1;

  // BME280（気圧・温湿度）
  float temp, hum, pres;
  bme.read(pres, temp, hum); // 順番注意: 気圧[Pa], 温度[℃], 湿度[%]

  // タイムスタンプ
  unsigned long t = millis();

  // シリアル出力（CSV形式）
  Serial.print(t); Serial.print(",");
  Serial.print(ax); Serial.print(","); Serial.print(ay); Serial.print(","); Serial.print(az); Serial.print(",");
  Serial.print(gx); Serial.print(","); Serial.print(gy); Serial.print(","); Serial.print(gz); Serial.print(",");
  Serial.print(mx); Serial.print(","); Serial.print(my); Serial.print(","); Serial.print(mz); Serial.print(",");
  Serial.print(distance); Serial.print(",");
  Serial.print(pres, 0); Serial.print(",");  // Pa単位整数
  Serial.print(temp, 1); Serial.print(",");
  Serial.println(hum, 1);                    // 改行

  delay(100); // 100msごとに送信（10Hz）
}
