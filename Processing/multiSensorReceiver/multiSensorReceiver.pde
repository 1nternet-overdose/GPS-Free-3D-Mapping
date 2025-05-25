import processing.serial.*;

Serial myPort;
String[] sensorData;
String[] latestData;

String filename = "sensor_log_" + year() + "_" + month() + "_" + day() + "_" + hour() + "_" + minute() + ".csv";
PrintWriter output;

// 歩数カウント関連
int stepCount = 0;
float stepLength = 0.65;
float totalDistance = 0;

// accMagバッファ
final int N = 100;
float[] accBuffer = new float[N];
float[] filtered = new float[N];
int accIndex = 0;

// FIRフィルタ係数
float[] firCoeff = {
  -0.0071, -0.0101, -0.0081, 0.0000, 0.0142,
   0.0316, 0.0478, 0.0583, 0.0583, 0.0478,
   0.0316, 0.0142, 0.0000, -0.0081, -0.0101,
  -0.0071
};

// 歩行ピーク検出
int lastStepTime = 0;
final int minStepInterval = 300;
final int stepDetectionDelay = 2000;
boolean stepDetectionEnabled = false;

// ZUPT静止判定
boolean ZUPT_flag = false;
int zuptCount = 0;
int zuptReleaseCount = 0;
final int ZUPT_COUNT_THRESHOLD = 20;
final int ZUPT_RELEASE_THRESHOLD = 5;
final float ZUPT_LOW = 0.96;
final float ZUPT_HIGH = 1.04;

float currentAccMag = 0;

void setup() {
  size(800, 400);
  printArray(Serial.list());

  String portName = Serial.list()[0];
  myPort = new Serial(this, portName, 115200);
  myPort.bufferUntil('\n');

  output = createWriter(filename);
  background(0);
  textSize(16);
  fill(255);
}

void draw() {
  background(0);
  if (latestData != null && latestData.length >= 14) {
    text("Steps: " + stepCount, 50, 30);
    text("Distance: " + nf(stepCount * stepLength, 1, 2) + " m", 50, 60);

    text("accMag = " + nf(currentAccMag, 1, 4) + " G", 50, 90);

    fill(ZUPT_flag ? color(0, 255, 0) : color(255, 0, 0));
    text("ZUPT: " + (ZUPT_flag ? "ON" : "OFF"), 50, 120);
    fill(255);

    stroke(0, 255, 0);
    for (int i = 1; i < N; i++) {
      float y1 = map(filtered[(accIndex + i - 1) % N], -1.5, 1.5, height, 0);
      float y2 = map(filtered[(accIndex + i) % N], -1.5, 1.5, height, 0);
      line(i - 1, y1, i, y2);
    }
  }
}

void serialEvent(Serial p) {
  String line = p.readStringUntil('\n');
  if (line == null) return;

  line = trim(line);
  sensorData = split(line, ',');
  if (sensorData.length < 14) return;
  output.println(line);
  latestData = sensorData;

  float ax = float(sensorData[1]);
  float ay = float(sensorData[2]);
  float az = float(sensorData[3]);
  float accMag = sqrt(ax * ax + ay * ay + az * az) / 16384.0;
  currentAccMag = accMag;

  // ZUPT判定
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

  // バッファ更新とフィルタ
  accBuffer[accIndex] = accMag;
  filtered[accIndex] = 0;
  for (int i = 0; i < firCoeff.length; i++) {
    int j = (accIndex - i + N) % N;
    filtered[accIndex] += accBuffer[j] * firCoeff[i];
  }

  // バッファ初期化完了後に歩行検出有効化
  if (!stepDetectionEnabled && accIndex > firCoeff.length) {
    stepDetectionEnabled = true;
  }

  // 歩行検出(誤検出防止)
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
    stepCount++;
    lastStepTime = now;
  }

  accIndex = (accIndex + 1) % N;
}

void keyPressed() {
  if (key == 'q' || key == 'Q') {
    output.flush();
    output.close();
    exit();
  }
}
