// シリアル通信，HTTP接続準備
import processing.serial.*;
import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import ddf.minim.analysis.*;

Serial myPort;

// ログ(センサデータ，特微量)ファイル準備
String[] sensorData;
String[] latestData;

// ファイル名の動的生成
String filename = "sensor_log_" + year() + "_" + month() + "_" + day() + "_" + hour() + "_" + minute() + ".csv";
PrintWriter output;
String featureFile = "step_features_" + year() + "_" + month() + "_" + day() + "_" + hour() + "_" + minute() + ".csv";
PrintWriter featureOut;

// 歩数，距離，バッファ等初期化：accMag，ステップ検出バッファ
int stepCount = 0;
float totalDistance = 0;
float lastStepLength = 0;

final int N = 100;
float[] accBuffer = new float[N];
float[] filtered = new float[N];
int accIndex = 0;

// FIRフィルタ係数定義：accMagノイズ低減(平滑化)
float[] firCoeff = {
  -0.0071, -0.0101, -0.0081, 0.0000, 0.0142,
   0.0316, 0.0478, 0.0583, 0.0583, 0.0478,
   0.0316, 0.0142, 0.0000, -0.0081, -0.0101,
  -0.0071
};

// ステップ検出条件設定：遅延，間隔
int lastStepTime = 0;
final int minStepInterval = 300;      // 最短歩行間隔
final int stepDetectionDelay = 2000;  // 起動後検出猶予
boolean stepDetectionEnabled = false; // 歩行検出ONフラグ

// ZUPT(静止判定)に関する設定
boolean ZUPT_flag = false;
int zuptCount = 0;
int zuptReleaseCount = 0;
final int ZUPT_COUNT_THRESHOLD = 20;  // 静止と判定する連続回数
final int ZUPT_RELEASE_THRESHOLD = 5; // 動き出し判定の連続回数
final float ZUPT_LOW = 0.96;          // accMagの下限
final float ZUPT_HIGH = 1.04;         // accMagの上限

float currentAccMag = 0;
float accMax = -9999;
float accMin = 9999;
int zuptStartTime = 0;
int zuptDuration = 0; // 静止時間

// マップ描画用の座標変数と方位角
float posX = 400, posY = 200;
float headingDeg = 0;
boolean shouldStep = false;

// FFTに関する設定
final int FFT_N = 128;                // サンプル数
float[] fftBuffer = new float[FFT_N]; // FFT用バッファ
int fftIndex = 0;                     // インデックス
float accEnergy = 0;                  // 加速度信号のエネルギー量
float dominantFreq = 0;               // 卓越周波数
FFT fft;

// EKFに関する設定
float theta = 0;                // 姿勢角(傾き角など)
float gyroBias = 0;             // バイアス(ゼロ点誤差)
float dt = 0.02;                // サンプリング周期(秒)：20ms≒50Hz
float P = 1, Q = 0.01, R = 0.5; // EKFパラメータ：P=誤差共分散(初期値), Q=プロセスノイズ(モデルの信頼度). R=観測ノイズ(センサ値の信頼度)

// 画面・ポート初期化：GUI構築，ファイル書き出し
void setup() {
  size(800, 400);
  printArray(Serial.list());
  String portName = Serial.list()[0];
  myPort = new Serial(this, portName, 115200);
  myPort.bufferUntil('\n');

  output = createWriter(filename);        // センサログファイル
  featureOut = createWriter(featureFile); // 特微量ログファイル
  featureOut.println("time,period,accMax,accMin,accAmp,zuptDuration,accEnergy,dominantFreq,estimatedLength,heading");

  fft = new FFT(FFT_N, 50);

  background(255);
  stroke(0);
  fill(0);
  textSize(14);
}

// マップ描画，ステップごとの線分：Processing描画API使用
void draw() {
  fill(255);
  noStroke();
  rect(0, 0, width, 50);
  fill(0);
  text("Steps: " + stepCount + "  Distance: " + nf(totalDistance, 1, 2) + " m", 20, 30);

  if (shouldStep) {
    float headingRad = radians(headingDeg);
    float dx = cos(headingRad) * lastStepLength * 100;
    float dy = sin(headingRad) * lastStepLength * 100;
    float newX = posX + dx;
    float newY = posY - dy;

    stroke(0, 0, 255);
    line(posX, posY, newX, newY);
    fill(255, 0, 0);
    ellipse(newX, newY, 5, 5);

    posX = newX;
    posY = newY;
    shouldStep = false;
  }
}

// センサデータ処理：配列に変換
void serialEvent(Serial p) {
  String line = p.readStringUntil('\n');
  if (line == null) return;

  line = trim(line);
  sensorData = split(line, ',');
  if (sensorData.length < 16) return;
  output.println(line);
  latestData = sensorData;

  float ax = float(sensorData[1]);
  float ay = float(sensorData[2]);
  float az = float(sensorData[3]);
  float gx = float(sensorData[4]);
  float gy = float(sensorData[5]);
  float gz = float(sensorData[6]);
  float mx = float(sensorData[7]);
  float my = float(sensorData[8]);
  float mz = float(sensorData[9]);

  // accMag計算：√(x^2+y^2+z^2)/16384
  float accMag = sqrt(ax * ax + ay * ay + az * az) / 16384.0;
  currentAccMag = accMag;

  // FFT：歩行周期の周波数検出
  fftBuffer[fftIndex % FFT_N] = accMag;
  fftIndex++;

  if (fftIndex >= FFT_N) {
    accEnergy = 0;
    for (int i = 0; i < FFT_N; i++) accEnergy += fftBuffer[i] * fftBuffer[i];
    accEnergy *= dt;

    fft.forward(fftBuffer);
    float maxVal = -1;
    int maxIndex = -1;
    for (int i = 1; i < fft.specSize(); i++) {
      if (fft.getBand(i) > maxVal) {
        maxVal = fft.getBand(i);
        maxIndex = i;
      }
    }
    dominantFreq = fft.indexToFreq(maxIndex);
  }

  // EKF：磁気センサとジャイロによるヨー角推定
  float magYaw = degrees(atan2(my, mx));
  float gyroYawRate = gz / 131.0; // MPU6050の感度係数

  theta += (gyroYawRate - gyroBias) * dt;
  P += Q;
  float y = magYaw - theta;
  float K = P / (P + R);
  theta += K * y;
  P *= (1 - K);
  headingDeg = theta;

  // accMax/accMin更新：振幅検出
  accMax = max(accMax, accMag);
  accMin = min(accMin, accMag);

  // ZUPT検出(静止状態判定)
  if (ZUPT_flag && zuptStartTime == 0) zuptStartTime = millis();
  if (!ZUPT_flag && zuptStartTime > 0) {
    zuptDuration = millis() - zuptStartTime;
    zuptStartTime = 0;
  }

  if (accMag >= ZUPT_LOW && accMag <= ZUPT_HIGH) {
    zuptCount++;
    if (zuptCount >= ZUPT_COUNT_THRESHOLD) {
      ZUPT_flag = true;
      zuptReleaseCount = 0;
    }
  } else {
    if (ZUPT_flag) {
      zuptReleaseCount++;
      if (zuptReleaseCount >= ZUPT_RELEASE_THRESHOLD) {
        ZUPT_flag = false;
        zuptCount = 0;
      }
    } else {
      zuptCount = 0;
    }
  }

  // FIRフィルタ処理(accMagの平滑化)
  accBuffer[accIndex] = accMag;
  filtered[accIndex] = 0;
  for (int i = 0; i < firCoeff.length; i++) {
    int j = (accIndex - i + N) % N;
    filtered[accIndex] += accBuffer[j] * firCoeff[i];
  }

  // ステップ検出有効化
  if (!stepDetectionEnabled && accIndex > firCoeff.length) {
    stepDetectionEnabled = true;
  }

  // 歩行のピーク検出
  int i1 = (accIndex - 1 + N) % N;
  int i2 = (accIndex - 2 + N) % N;
  int now = millis();

  if (
    stepDetectionEnabled &&
    now > stepDetectionDelay &&
    filtered[i1] > filtered[i2] &&
    filtered[i1] > filtered[accIndex] &&
    filtered[i1] > 0.274 &&
    now - lastStepTime > minStepInterval
  ) {

    // ステップ検出成立
    int stepInterval = now - lastStepTime;
    float accAmp = accMax - accMin;

    // 歩幅推定
    float predictedLength = sendForPrediction(stepInterval, accMax, accMin, accAmp, zuptDuration, accEnergy, dominantFreq);

    // 各種更新
    lastStepLength = predictedLength;
    totalDistance += lastStepLength;
    stepCount++;
    lastStepTime = now;
    shouldStep = true;

    // 特徴量ログ書き出し
    featureOut.println(now + "," + stepInterval + "," + nf(accMax, 1, 4) + "," + nf(accMin, 1, 4) + "," + nf(accAmp, 1, 4) + "," + zuptDuration + "," + nf(accEnergy, 1, 3) + "," + nf(dominantFreq, 1, 2) + "," + nf(predictedLength, 1, 3) + "," + headingDeg);
    featureOut.flush();

    accMax = -9999;
    accMin = 9999;
    zuptDuration = 0;
  }

  // accIndex更新：リングバッファ
  accIndex = (accIndex + 1) % N;
}

// 機械学習API呼び出し
float sendForPrediction(float period, float accMax, float accMin, float accAmp, int zuptDuration, float accEnergy, float dominantFreq) {
  try {
    // 特徴量をJSON形式で作成
    JSONObject json = new JSONObject();
    json.setFloat("period", period);
    json.setFloat("accMax", accMax);
    json.setFloat("accMin", accMin);
    json.setFloat("accAmp", accAmp);
    json.setInt("zuptDuration", zuptDuration);
    json.setFloat("accEnergy", accEnergy);
    json.setFloat("dominantFreq", dominantFreq);

    // HTTP POSTでFlaskへ送信
    URL url = new URL("http://127.0.0.1:5000/predict");
    HttpURLConnection conn = (HttpURLConnection) url.openConnection();
    conn.setRequestMethod("POST");
    conn.setRequestProperty("Content-Type", "application/json");
    conn.setDoOutput(true);

    OutputStream os = conn.getOutputStream();
    os.write(json.toString().getBytes("utf-8"));
    os.flush();
    os.close();

    // 応答取得(JSON)
    String response = "";
    BufferedReader br = new BufferedReader(new InputStreamReader(conn.getInputStream()));
    String line;
    while ((line = br.readLine()) != null) {
      response += line;
    }
    br.close();

    JSONObject res = parseJSONObject(response);
    return res.getFloat("stepLength");
  } catch (Exception e) {
    println("[ERROR] ML prediction failed: " + e.getMessage());
    return 0.6;
  }
}

// 終了処理
void keyPressed() {
  if (key == 'q' || key == 'Q') {
    output.flush();
    output.close();
    featureOut.flush();
    featureOut.close();
    exit();
  }
}
