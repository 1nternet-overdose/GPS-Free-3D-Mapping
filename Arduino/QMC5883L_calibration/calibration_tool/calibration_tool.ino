#include <Wire.h>
#include <QMC5883LCompass.h>

QMC5883LCompass compass;

// 最小・最大値の初期化
int x_min = 32767, x_max = -32768;
int y_min = 32767, y_max = -32768;
int z_min = 32767, z_max = -32768;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  compass.init();
  Serial.println("QMC5883L キャリブレーション開始");
  Serial.println("センサを全方向にゆっくり回転させてください");
}

void loop() {
  compass.read();
  int x = compass.getX();
  int y = compass.getY();
  int z = compass.getZ();

  // 最小・最大値の更新
  if (x < x_min) x_min = x;
  if (x > x_max) x_max = x;
  if (y < y_min) y_min = y;
  if (y > y_max) y_max = y;
  if (z < z_min) z_min = z;
  if (z > z_max) z_max = z;

  // 100msごとに出力
  Serial.print("X: "); Serial.print(x_min); Serial.print(" ~ "); Serial.print(x_max); Serial.print("\t");
  Serial.print("Y: "); Serial.print(y_min); Serial.print(" ~ "); Serial.print(y_max); Serial.print("\t");
  Serial.print("Z: "); Serial.print(z_min); Serial.print(" ~ "); Serial.println(z_max);

  delay(100);
}
