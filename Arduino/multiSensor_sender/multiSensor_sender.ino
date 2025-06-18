// --- 必要ライブラリ ---
#include <Wire.h>
#include <MPU6050.h>
#include <QMC5883LCompass.h>
#include <VL53L0X.h>
#include <BME280I2C.h>
#include <math.h>

// --- センサインスタンス ---
MPU6050 mpu;
QMC5883LCompass compass;
VL53L0X tof;
BME280I2C bme;

// --- 加速度・角速度の物理量：float型 ---
float accMag;

// --- ZUPT検出フラグ ---
bool zuptFlag = false;
unsigned long lastZuptTime = 0;
const int zuptWindow = 200; // ms
const float zuptThreshold = 0.02; // ±G判定範囲（ノルム）

// --- センサ値増幅のための各種倍率 ---
const float accelGain = 2.0;    // 加速度の倍率
const float gyroGain = 2.0;     // ジャイロの倍率
// const float magGain  = 1.5;  // 地磁気の倍率

// --- ガウスフィルタ ---
const int KERNEL_SIZE = 5;
const float gaussianKernel[KERNEL_SIZE] = {0.06136, 0.24477, 0.38774, 0.24477, 0.06136};

float axHistory[KERNEL_SIZE] = {0}, ayHistory[KERNEL_SIZE] = {0}, azHistory[KERNEL_SIZE] = {0};
float gxHistory[KERNEL_SIZE] = {0}, gyHistory[KERNEL_SIZE] = {0}, gzHistory[KERNEL_SIZE] = {0};
float mxHistory[KERNEL_SIZE] = {0}, myHistory[KERNEL_SIZE] = {0};
float distHistory[KERNEL_SIZE] = {0}, presHistory[KERNEL_SIZE] = {0};

float applyGaussianFilter(float newVal, float* buffer, const float* kernel, int size) {
  for (int i = 0; i < size - 1; i++) buffer[i] = buffer[i + 1];
  buffer[size - 1] = newVal;
  float result = 0.0;
  for (int i = 0; i < size; i++) result += buffer[i] * kernel[i];
  return result;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  mpu.initialize(); // MPU6050 初期化
  compass.init();   // QMC5883L 初期化
  compass.setCalibration(-696, 1446, -1803, 98, -636, 1445);

  tof.setTimeout(500);
  // VL53L0X 初期化と連続測定モード開始
  if (!tof.init()) {
    Serial.println("VL53L0X failed");
    while (1);
  }
  tof.startContinuous();

  // BME280 初期化
  while (!bme.begin()) delay(500);
}

void loop() {
  // --- MPU6050 ---
  int16_t ax_raw, ay_raw, az_raw;
  int16_t gx_raw, gy_raw, gz_raw;

  mpu.getAcceleration(&ax_raw, &ay_raw, &az_raw); // 加速度(x, y, z)取得
  mpu.getRotation(&gx_raw, &gy_raw, &gz_raw);

  float ax = ay_raw / 16384.0;
  float ay = ax_raw / 16384.0;
  float az = az_raw / 16384.0;
  float gx = gy_raw / 131.0;
  float gy = gx_raw / 131.0;
  float gz = gz_raw / 131.0;

  float axF = applyGaussianFilter(ax, axHistory, gaussianKernel, KERNEL_SIZE) * accelGain;
  float ayF = applyGaussianFilter(ay, ayHistory, gaussianKernel, KERNEL_SIZE) * accelGain;
  float azF = applyGaussianFilter(az, azHistory, gaussianKernel, KERNEL_SIZE) * accelGain;

  float gxF = applyGaussianFilter(gx, gxHistory, gaussianKernel, KERNEL_SIZE) * gyroGain;
  float gyF = applyGaussianFilter(gy, gyHistory, gaussianKernel, KERNEL_SIZE) * gyroGain;
  float gzF = applyGaussianFilter(gz, gzHistory, gaussianKernel, KERNEL_SIZE) * gyroGain;

  accMag = sqrt(ax * ax + ay * ay + az * az);

  // --- ZUPT 判定 ---
  unsigned long now = millis();
  if (accMag > (2.7 - zuptThreshold) && accMag < (2.7 + zuptThreshold)) {
    if (now - lastZuptTime > zuptWindow) {
      zuptFlag = true;
      lastZuptTime = now;
    } else {
      zuptFlag = false;
    }
  } else {
    zuptFlag = false;
    lastZuptTime = now;
  }

  // --- QMC5883L ---
  compass.read();
  int mx = -compass.getX();
  int my = compass.getZ();

  float mxF = applyGaussianFilter(mx, mxHistory, gaussianKernel, KERNEL_SIZE);
  float myF = applyGaussianFilter(my, myHistory, gaussianKernel, KERNEL_SIZE);
  float magYaw = atan2(myF, mxF) * 180.0 / PI;
  if (magYaw < 0) magYaw += 360.0;

  // --- VL53L0X ---
  int distance = tof.readRangeContinuousMillimeters();  // mm単位
  float distF = applyGaussianFilter((float)(distance - 20), distHistory, gaussianKernel, KERNEL_SIZE);

  // --- BME280 ---
  float temp, hum, pres;
  bme.read(pres, temp, hum);  // 気圧・温度・湿度
  float presF = applyGaussianFilter(pres, presHistory, gaussianKernel, KERNEL_SIZE);

  // --- 送信 ---
  Serial.print("A,");   // パケット識別子
  Serial.print(axF, 3); Serial.print(",");
  Serial.print(ayF, 3); Serial.print(",");
  Serial.print(azF, 3); Serial.print(",");
  Serial.print(gxF, 3); Serial.print(",");
  Serial.print(gyF, 3); Serial.print(",");
  Serial.print(gzF, 3); Serial.print(",");
  Serial.print(accMag, 3); Serial.print(",");
  Serial.print(zuptFlag ? 1 : 0); Serial.print(",");
  Serial.print(magYaw, 2); Serial.print(",");
  Serial.print(distF, 2); Serial.print(",");
  Serial.print(presF, 2); Serial.println();

  delay(50);
}