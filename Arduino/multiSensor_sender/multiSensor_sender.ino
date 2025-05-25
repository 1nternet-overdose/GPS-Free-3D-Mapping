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

bool magReady = false;
int xStableCount = 0, yStableCount = 0, zStableCount = 0;
const int magStableThreshold = 10;

const int magDriftThreshold = 30; // [%]
const int magDriftRequired = 30;
int magDriftCount = 0;
const int MAG_HISTORY_LEN = 10;
float magHistory[MAG_HISTORY_LEN];
float magAvg = 0;
int magIndex = 0;

int minX = 32767, maxX = -32768;
int minY = 32767, maxY = -32768;
int minZ = 32767, maxZ = -32768;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  mpu.initialize();
  compass.init();
  vl53.init();
  vl53.setTimeout(500);
  vl53.startContinuous();
  bme.begin();
}

void loop() {
  // MPU6050
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // QMC5883L
  compass.read();
  int mx = compass.getX();
  int my = compass.getY();
  int mz = compass.getZ();

  // VL53L0X
  int distance = vl53.readRangeContinuousMillimeters();
  if (vl53.timeoutOccurred()) distance = -1;

  // BME280
  float temp, hum, pres;
  bme.read(pres, temp, hum);

  // 磁場の大きさ
  float magNorm = sqrt(mx * mx + my * my + mz * mz);

  // QMC5883Lキャリブレーションロジック
  if (!magReady) {
    // 各軸: 最大・最小の更新確認
    if (mx < minX) {
      minX = mx; xStableCount = 0;
    } else if (mx > maxX) {
      maxX = mx; xStableCount = 0;
    } else {
      xStableCount++;
    }

    if (my < minY) {
      minY = my; yStableCount = 0;
    } else if (my > maxY) {
      maxY = my; yStableCount = 0;
    } else {
      yStableCount++;
    }

    if (mz < minZ) {
      minZ = mz; zStableCount = 0;
    } else if (mz > maxZ) {
      maxZ = mz; zStableCount = 0;
    } else {
      zStableCount++;
    }

    // 全軸安定でキャリブレーション完了
    if (xStableCount >= magStableThreshold &&
        yStableCount >= magStableThreshold &&
        zStableCount >= magStableThreshold) {
      magReady = true;
    }

  } else {
    // 磁場変化を平均と比較してドリフト検知
    magAvg -= magHistory[magIndex] / MAG_HISTORY_LEN;
    magHistory[magIndex] = magNorm;
    magAvg += magNorm / MAG_HISTORY_LEN;
    magIndex = (magIndex + 1) % MAG_HISTORY_LEN;

    if (abs(magNorm - magAvg) > magAvg * (magDriftThreshold / 100.0)) {
      magDriftCount++;
    } else {
      magDriftCount = max(magDriftCount - 1, 0);
    }

    if (magDriftCount >= magDriftRequired) {
      // 再キャリブレーション
      magReady = false;
      xStableCount = yStableCount = zStableCount = 0;
      minX = minY = minZ = 32767;
      maxX = maxY = maxZ = -32768;
      magDriftCount = 0;
    }
  }

  // タイムスタンプ
  unsigned long t = millis();

  // シリアル出力(CSV形式)
  Serial.print(t); Serial.print(",");
  Serial.print(ax); Serial.print(","); Serial.print(ay); Serial.print(","); Serial.print(az); Serial.print(",");
  Serial.print(gx); Serial.print(","); Serial.print(gy); Serial.print(","); Serial.print(gz); Serial.print(",");
  Serial.print(mx); Serial.print(","); Serial.print(my); Serial.print(","); Serial.print(mz); Serial.print(",");
  Serial.print(distance); Serial.print(",");
  Serial.print(pres, 0); Serial.print(",");
  Serial.print(temp, 1); Serial.print(",");
  Serial.print(hum, 1); Serial.print(",");
  Serial.println(magReady ? 1 : 0);

  delay(100); // 100msごとに送信(10Hz)
}
