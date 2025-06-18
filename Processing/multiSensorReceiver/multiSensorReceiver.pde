// --- 必要ライブラリ ---
import processing.serial.*;
import java.util.*;
import java.io.PrintWriter;
import ddf.minim.analysis.*;
import ddf.minim.*;

// --- 通信・出力関連 ---
Serial myPort;
PrintWriter logWriter;

// --- 自己位置・角度・歩数関連 ---
float posX = 0, posY = 0, posZ = 0;
float theta = 0;
float stepLength = 0.6;
int stepCount = 0;
float totalDistance = 0.0;

// --- センサデータ ---
float accMag;
float magYaw;
int distance;
float pressure;
boolean zupt = false;
String climbDirection = "";

// --- センサ履歴（時系列バッファ） ---
final int MAX_HISTORY = 300;
ArrayList<Float> accXHistory = new ArrayList<Float>();
ArrayList<Float> accYHistory = new ArrayList<Float>();
ArrayList<Float> accZHistory = new ArrayList<Float>();
ArrayList<Float> pressureHistory = new ArrayList<Float>();

// --- コピー用（並列処理などで使用） ---
ArrayList<Float> accXHistoryCopy, accYHistoryCopy, accZHistoryCopy;

// --- ZUPT（静止判定）用 ---
final int ZUPT_THRESHOLD_FRAME = 10;
final float ZUPT_DELTA_THRESHOLD = 0.02;
final int ZUPT_STABLE_WINDOW = 10;
int zuptFrame = 0;
ArrayList<Float> accMagWindow = new ArrayList<Float>();

// --- 軌跡・障害物ログ ---
ArrayList<PVector> trajectory = new ArrayList<PVector>();
ArrayList<PVector> obstacles = new ArrayList<PVector>();

// --- 障害物表示ウィンドウ ---
ObstacleWindow obstacleWin;

// --- クールダウン変数 ---
int stepCooldown = 0;
int climbCooldown = 0;

// --- 前回ステップ検出時刻 ---
int lastStepTime = 0;

// --- ビープ音関連 ---
Minim minim;
AudioOutput out;
BeepTone beepTone;
boolean isBeeping = false;
int beepCooldown = 0;
int validDistanceMin = 0;     // 最小0m
int validDistanceMax = 2000;  // 最大2m
int beepStartTime = 0;
int beepInterval = 1000;
int beepDuration = 50;

void setup() {
  size(800, 800, P3D);
  myPort = new Serial(this, Serial.list()[0], 115200);
  myPort.bufferUntil('\n');
  background(255);
  textSize(14);

  String fileName = "motion_log_" + year() + nf(month(), 2) + nf(day(), 2) + "_" + nf(hour(), 2) + nf(minute(), 2) + ".csv";
  logWriter = createWriter(fileName);
  logWriter.println("time,posX,posY,posZ,theta,stepCount,totalDistance,climbDirection,zupt,accX,accY,accZ");
  
  // --- Minim の初期化 ---
  minim = new Minim(this);
  out = minim.getLineOut();
  beepTone = new BeepTone(1000, 0.5, out.sampleRate());
  out.addSignal(beepTone);
  beepTone.setAmplitude(0);  // 初期はミュート

  // obstacleWin = new ObstacleWindow();
  // String[] args = {"ObstacleWindow"};
  // PApplet.runSketch(args, obstacleWin);
}

void draw() {
  synchronized (accXHistory) {
    accXHistoryCopy = new ArrayList<Float>(accXHistory);
  }
  synchronized (accYHistory) {
    accYHistoryCopy = new ArrayList<Float>(accYHistory);
  }
  synchronized (accZHistory) {
    accZHistoryCopy = new ArrayList<Float>(accZHistory);
  }

  background(255);
  lights();
  translate(width / 2, height / 2, 620);
  rotateX(radians(-30));
  rotateY(radians(-60));
  rotateZ(radians(0));
  drawGridBox(10, 50);

  stroke(0, 0, 255);
  strokeWeight(1.5);
  noFill();
  beginShape();
  synchronized (trajectory) {
    for (PVector p : trajectory) {
      vertex(p.x, p.y, p.z);
    }
  }
  endShape();

  fill(0);
  hint(DISABLE_DEPTH_TEST);
  camera();
  textAlign(LEFT);
  text("ZUPT: " + zupt, 20, 20);
  text("Step Count: " + stepCount, 20, 40);
  text("Total Distance: " + nf(totalDistance, 1, 2) + " m", 20, 60);
  text("Climb Direction: " + climbDirection, 20, 80);
  //text("ToF Distance: " + distance + " mm", 20, 100);

  drawWave(accZHistoryCopy, color(0, 200, 255), 140, "Z axis (Step Acc)");
  drawWave(accYHistoryCopy, color(0, 255, 100), 200, "Y axis (Climb Acc)");
  drawWave(accXHistoryCopy, color(255, 100, 100), 260, "X axis (Turn Acc)");
  drawCompass(theta, 100, 700, 40);

  hint(ENABLE_DEPTH_TEST);

  detectMotionAndLog();
  
  // --- ToFデバッグ表示と音制御 ---
  fill(0);
  textAlign(LEFT);
  if (distance < validDistanceMin || distance > validDistanceMax) {
    text("ToF Distance: OUT OF RANGE", 20, 100);
    beepTone.setAmplitude(0);     // ミュート
    isBeeping = false;
    beepStartTime = millis();     // リセット：連続誤検知防止
  } else {
    text("ToF Distance: " + nf(distance / 10.0, 1, 1) + " cm", 20, 100);

    // 音の間隔：距離に応じて条件分岐）
    int interval;
    if (distance <= 100) {          // ～10cm未満：連続
      interval = 100;
    } else if (distance <= 300) {   // ～30cm：速く
      interval = 400;
    } else if (distance <= 500) {   // ～50cm：遅く
      interval = 1000;
    } else {                        // ～2m：それ以上は鳴らさない
      beepTone.setAmplitude(0);
      isBeeping = false;
      beepStartTime = millis();
      return;
    }

    // ビープ制御：非ブロッキング
    if (!isBeeping && millis() - beepStartTime > interval) {
      beepTone.setAmplitude(0.5);
      isBeeping = true;
      beepStartTime = millis();  // 鳴らし始めの時刻記録
    }

    if (isBeeping && millis() - beepStartTime > 50) {
      beepTone.setAmplitude(0.0);
      isBeeping = false;
    }
  }
}

void serialEvent(Serial myPort) {
  String inData = trim(myPort.readStringUntil('\n'));
  if (inData == null || !inData.startsWith("A")) return;
  String[] vals = split(inData, ',');
  if (vals.length < 12) return;

  try {
    float accX = float(vals[1]);
    float accY = float(vals[2]);
    float accZ = float(vals[3]);
    accMag = float(vals[7]);
    magYaw = float(vals[9]);
    distance = int(vals[10]);
    pressure = float(vals[11]);

    theta = magYaw;

    float accNorm = sqrt(accX * accX + accY * accY + accZ * accZ);
    accMagWindow.add(accNorm);
    if (accMagWindow.size() > ZUPT_STABLE_WINDOW) accMagWindow.remove(0);

    float mean = 0;
    for (float v : accMagWindow) mean += v;
    mean /= accMagWindow.size();

    float sd = 0;
    for (float v : accMagWindow) sd += sq(v - mean);
    sd = sqrt(sd / accMagWindow.size());

    if (sd < ZUPT_DELTA_THRESHOLD) {
      zuptFrame++;
      if (zuptFrame >= ZUPT_THRESHOLD_FRAME) zupt = true;
    } else {
      zuptFrame = 0;
      zupt = false;
    }

    synchronized (accXHistory) {
      if (accXHistory.size() > MAX_HISTORY) accXHistory.remove(0);
      accXHistory.add(accX);
    }
    synchronized (accYHistory) {
      if (accYHistory.size() > MAX_HISTORY) accYHistory.remove(0);
      accYHistory.add(accY);
    }
    synchronized (accZHistory) {
      if (accZHistory.size() > MAX_HISTORY) accZHistory.remove(0);
      accZHistory.add(accZ);
    }

    if (distance > 20 && distance < 3000) {
      float dx = cos(radians(theta)) * distance / 100.0;
      float dy = sin(radians(theta)) * distance / 100.0;
      PVector obs = new PVector(posX + dx, posY + dy, posZ);
      synchronized (obstacles) {
        obstacles.add(obs);
        if (obstacles.size() > 100) obstacles.remove(0);
      }
    }

  } catch (Exception e) {
    println("Parse error: " + e.getMessage());
  }
}

void detectMotionAndLog() {
  int N = 10;
  if (accZHistoryCopy.size() < N || accXHistoryCopy.size() < N || accYHistoryCopy.size() < N) return;

  // --- 歩行検出：Z軸 ---
  if (!zupt && stepCooldown == 0) {
    float maxZ = -Float.MAX_VALUE;
    float minZ = Float.MAX_VALUE;
    for (int i = accZHistoryCopy.size() - N; i < accZHistoryCopy.size(); i++) {
      float val = accZHistoryCopy.get(i);
      if (val > maxZ) maxZ = val;
      if (val < minZ) minZ = val;
    }

    float deltaZ = maxZ - minZ;

    if (deltaZ > 1.0) {
      stepLength = estimateStepLength(accZHistoryCopy, deltaZ);
      float dz = stepLength * cos(radians(theta));
      float dx = stepLength * sin(radians(theta));
      posZ += dz;
      posX += dx;
      totalDistance += stepLength;
      stepCount++;
      
      trajectory.add(new PVector(posX, posY, posZ));
      
      println("Step Detected: " + stepCount + " (" + nf(stepLength, 1, 2) + " m)");
      
      stepCooldown = 15;
    }
  } else if (stepCooldown > 0) {
    stepCooldown--;
  }

  // --- 昇降検出：X軸 ---
  if (!zupt && climbCooldown == 0 && accXHistoryCopy.size() >= 64) {
    float maxX = -Float.MAX_VALUE;
    float minX = Float.MAX_VALUE;
    float sumX = 0;

    for (int i = accXHistoryCopy.size() - N; i < accXHistoryCopy.size(); i++) {
      float val = accXHistoryCopy.get(i);
      maxX = max(maxX, val);
      minX = min(minX, val);
      sumX += val;
    }

    float deltaX = maxX - minX;
    float avgX = sumX / N;

    println("avgX: " + nf(avgX, 1, 3));

    // FFTで昇降周期を推定
    float[] fftInput = new float[64];
    for (int i = 0; i < 64; i++) {
      fftInput[i] = accXHistoryCopy.get(accXHistoryCopy.size() - 64 + i);
    }
    float freq = estimateFrequency(fftInput, 30.0);  // 30Hzサンプリング

    // 周期に応じて昇降量を調整：速い動きほど大きく昇降
    float stepHeight = constrain(map(freq, 0.5, 3.0, 0.05, 0.2), 0.05, 0.2);

    if (deltaX > 0.5) {
      if (avgX < 1.0) {
        climbDirection = "Climbing";
        posY -= stepHeight;
        trajectory.add(new PVector(posX, posY, posZ));
        println("Climb Detected ↑ (" + nf(stepHeight, 1, 3) + " m)");
      } else if (avgX > 1.5) {
        climbDirection = "Descending";
        posY += stepHeight;
        trajectory.add(new PVector(posX, posY, posZ));
        println("Climb Detected ↓ (" + nf(stepHeight, 1, 3) + " m)");
      } else {
        climbDirection = "Stable";
      }
      climbCooldown = 15;
    }
  } else if (climbCooldown > 0) {
    climbCooldown--;
  }

  // --- ログ出力（fx, fy, fz ⇒ 最新の生加速度） ---
  float latestX = accXHistoryCopy.get(accXHistoryCopy.size() - 1);
  float latestY = accYHistoryCopy.get(accYHistoryCopy.size() - 1);
  float latestZ = accZHistoryCopy.get(accZHistoryCopy.size() - 1);

  logWriter.println(millis() + "," +
                    nf(posX, 1, 2) + "," +
                    nf(posY, 1, 2) + "," +
                    nf(posZ, 1, 2) + "," +
                    nf(theta, 1, 2) + "," +
                    stepCount + "," +
                    nf(totalDistance, 1, 2) + "," +
                    climbDirection + "," +
                    zupt + "," +
                    nf(latestX, 1, 3) + "," +
                    nf(latestY, 1, 3) + "," +
                    nf(latestZ, 1, 3));
  logWriter.flush();
}

float estimateStepLength(ArrayList<Float> accZHistory, float deltaZ) {
  int currentTime = millis();
  float intervalSec = (currentTime - lastStepTime) / 1000.0;
  lastStepTime = currentTime;

  // FFTが可能なだけの履歴があるかを確認
  float freq = 1.2; // 初期値
  if (accZHistory.size() >= 64) {
    float[] fftInput = new float[64];
    int start = accZHistory.size() - 64;
    for (int i = 0; i < 64; i++) {
      fftInput[i] = accZHistory.get(start + i);
    }
    freq = estimateFrequency(fftInput, 30.0);
  }

  // 各成分
  float ampComponent  = constrain(map(deltaZ, 1.0, 4.0, 0.4, 0.8), 0.3, 1.0);
  float freqComponent = constrain(map(freq, 0.8, 2.5, 0.4, 0.9), 0.3, 1.0);
  float timeComponent = constrain(map(intervalSec, 0.3, 1.2, 0.9, 0.4), 0.3, 1.0);

  // 加重平均で歩幅決定
  return (ampComponent * 0.4 + freqComponent * 0.3 + timeComponent * 0.3);
}

float estimateFrequency(float[] signal, float fps) {
  FFT fft = new FFT(signal.length, fps);
  fft.forward(signal);

  int maxIndex = 1;
  float maxValue = fft.getBand(1);
  for (int i = 2; i < fft.specSize(); i++) {
    float value = fft.getBand(i);
    if (value > maxValue) {
      maxValue = value;
      maxIndex = i;
    }
  }
  return fft.indexToFreq(maxIndex);
}

void drawWave(ArrayList<Float> data, int col, int yOffset, String label) {
  stroke(col);
  noFill();
  beginShape();
  int n = data.size();
  for (int i = 0; i < n; i++) {
    float x = map(i, 0, n - 1, 20, width - 20); // 横軸：データインデックス
    float y = map(data.get(i), -2, 2, height - yOffset, height - yOffset - 50); // 縦軸：加速度値
    vertex(x, y);
  }
  endShape();
  
  fill(col);
  textAlign(LEFT, BOTTOM);
  text(label, 20, height - yOffset - 55);
}

void drawGridBox(int spacing, int size) {
  stroke(200);
  strokeWeight(1);
  for (int i = -size; i <= size; i += spacing) {
    for (int j = -size; j <= size; j += spacing) {
      line(i, j, -size, i, j, size);
      line(-size, j, i, size, j, i);
      line(i, -size, j, i, size, j);
    }
  }
}

void drawCompass(float angleDeg, int centerX, int centerY, int size) {
  pushMatrix();
  translate(centerX, centerY);
  
  // 外円
  stroke(0);
  fill(250);
  ellipse(0, 0, size * 2, size * 2);
  
  // 進行方向
  float angleRad = radians(angleDeg);
  float x = size * cos(angleRad - HALF_PI);
  float y = size * sin(angleRad - HALF_PI);
  stroke(255, 0, 0);
  strokeWeight(3);
  line(0, 0, x, y);
  
  // 回転角度表示
  fill(0);
  textAlign(CENTER, CENTER);
  text(nf(angleDeg, 1, 1) + "°", 0, 0);
  
  // 方位表示（非表示）
  // text("N", 0, -size + 10);
  // text("S", 0, size - 10);
  // text("W", -size + 10, 0);
  // text("E", size - 10, 0);
  
  popMatrix();
}

// --- ビープ音生成：AudioSignal ---
class BeepTone implements AudioSignal {
  float frequency;
  float amplitude;
  float phase;
  float sampleRate;

  BeepTone(float freq, float amp, float rate) {
    frequency = freq;
    amplitude = amp;
    sampleRate = rate;
    phase = 0;
  }

  void setAmplitude(float amp) {
    amplitude = amp;
  }

  void setFrequency(float freq) {
    frequency = freq;
  }

  // 音波生成関数：矩形波
  public void generate(float[] left, float[] right) {
    for (int i = 0; i < left.length; i++) {
      float wave = (sin(TWO_PI * frequency * phase / sampleRate) > 0) ? amplitude : -amplitude;
      left[i] = wave;
      right[i] = wave;
      phase++;
      if (phase >= sampleRate) phase = 0;
    }
  }

  public void generate(float[] signal) {
    for (int i = 0; i < signal.length; i++) {
      float wave = (sin(TWO_PI * frequency * phase / sampleRate) > 0) ? amplitude : -amplitude;
      signal[i] = wave;
      phase++;
      if (phase >= sampleRate) phase = 0;
    }
  }
}

// --- サブウィンドウ：障害物3D表示 ---
public class ObstacleWindow extends PApplet {
  public void settings() {
    size(600, 600, P3D);
  }

  public void setup() {
    surface.setTitle("障害物3D表示");
  }
  
  public void draw() {
    background(240);
    lights();

    // 一人称視点カメラ
    float scale = 20.0;
    float eyeX = posX * scale;
    float eyeY = -posY * scale;
    float eyeZ = -posZ * scale + 50;

    float centerX = eyeX + 100 * sin(radians(theta));
    float centerY = eyeY;
    float centerZ = eyeZ + 100 * cos(radians(theta));

    camera(eyeX, eyeY, eyeZ, centerX, centerY, centerZ, 0, 1, 0);

    // 座標軸ガイド
    strokeWeight(2);
    stroke(255, 0, 0); line(0, 0, 0, 50, 0, 0); // X軸：赤
    stroke(0, 255, 0); line(0, 0, 0, 0, 50, 0); // Y軸：緑
    stroke(0, 0, 255); line(0, 0, 0, 0, 0, 50); // Z軸：青

    fill(180, 50, 50);
    stroke(100);
    synchronized (obstacles) {
      for (PVector obs : obstacles) {
        pushMatrix();
        translate(obs.x * scale, -obs.y * scale, -obs.z * scale);
        box(10);
        popMatrix();
      }
    }

    // HUD表示
    hint(DISABLE_DEPTH_TEST);
    camera(); // デフォルトカメラに戻してテキスト表示
    fill(0);
    textSize(12);
    text("Obstacle Count: " + obstacles.size(), 20, 20);
    hint(ENABLE_DEPTH_TEST);
  }
}