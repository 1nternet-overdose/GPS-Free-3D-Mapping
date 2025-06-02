#include <Wire.h>

// 各種センサライブラリのインクルード
#include <MPU6050.h>
#include <QMC5883LCompass.h>
#include <VL53L0X.h>
#include <BME280I2C.h>

// センサインスタンス生成
MPU6050 mpu;
QMC5883LCompass compass;
VL53L0X vl53;
BME280I2C bme;

// 地磁気センサキャリブレーション用変数定義：地磁気安定性確認，ドリフト判定
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

// 各センサ初期化処理
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
  // MPU6050：加速度・ジャイロ取得
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // QMC5883L：地磁気取得
  compass.read();
  int mx = compass.getX();
  int my = compass.getY();
  int mz = compass.getZ();
  int heading = compass.getAzimuth();

  // VL53L0X：距離取得
  int distance = vl53.readRangeContinuousMillimeters();
  if (vl53.timeoutOccurred()) distance = -1;

  // BME280：温度・湿度・気圧取得
  float temp, hum, pres;
  bme.read(pres, temp, hum);

  // 地磁気ベクトルの大きさ：√(x^2+y^2+z^2)
  float magNorm = sqrt(mx * mx + my * my + mz * mz);

  // キャリブレーション状態管理(初期化完了条件)：min/maxの安定判定
  if (!magReady) {
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

    if (xStableCount >= magStableThreshold &&
        yStableCount >= magStableThreshold &&
        zStableCount >= magStableThreshold) {
      magReady = true;
    }

  } else {
    // 移動平均の更新：magNormの履歴を保持し平均値を算出
    magAvg -= magHistory[magIndex] / MAG_HISTORY_LEN;
    magHistory[magIndex] = magNorm;
    magAvg += magNorm / MAG_HISTORY_LEN;
    magIndex = (magIndex + 1) % MAG_HISTORY_LEN;

    // ドリフト判定：現在値と平均値の差分検出
    if (abs(magNorm - magAvg) > magAvg * (magDriftThreshold / 100.0)) {
      magDriftCount++;
    } else {
      magDriftCount = max(magDriftCount - 1, 0);
    }

    // 再キャリブレーションの実行条件と処理
    if (magDriftCount >= magDriftRequired) {
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
  Serial.print(magReady ? 1 : 0); Serial.print(",");
  Serial.println(heading); // Azimuth (0-360 degrees)

  // 10Hzサンプリング：100ms周期で送信
  delay(100);
}