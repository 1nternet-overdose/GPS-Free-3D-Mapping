// --- 必要ライブラリ ---
import processing.serial.*;
import java.util.*;
import java.io.PrintWriter;
import ddf.minim.analysis.*;
import ddf.minim.*;

// --- 通信・出力関連 ---
Serial myPort;
PrintWriter logWriter;

// --- センサデータ ---
float accMag;
float magYaw;
int distance;
float pressure;
boolean zupt = false;
String climbDirection = "";

// --- 処理遅延 ---
int arduinoSentTime = 0;
int systemDelay = 0;

// --- 自己位置・角度・歩数関連 ---
float posX = 0, posY = 0, posZ = 0;   // 自己位置
float theta = 0;                      // 進行方向（角度）
float stepLength = 17.2;
int stepCount = 0;
int climbUpCount = 0;
int climbDownCount = 0;
float totalDistance = 0.0;
int lastStepTime = 0;                 // 前回ステップ検出時刻
int stepCooldown = 0;
int climbCooldown = 0;
boolean stepDetected = false;

// --- 軌跡・障害物ログ ---
ArrayList<PVector> trajectory = new ArrayList<PVector>();
ArrayList<PVector> obstacles = new ArrayList<PVector>();

// --- ZUPT（静止判定）関連 ---
final int ZUPT_THRESHOLD_FRAME = 10;
final float ZUPT_DELTA_THRESHOLD = 0.02;
final int ZUPT_STABLE_WINDOW = 10;
int zuptFrame = 0;
ArrayList<Float> accMagWindow = new ArrayList<Float>();

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

// --- センサ履歴用時系列バッファ ---
final int MAX_HISTORY = 300;
ArrayList<Float> accXHistory = new ArrayList<Float>();
ArrayList<Float> accYHistory = new ArrayList<Float>();
ArrayList<Float> accZHistory = new ArrayList<Float>();
ArrayList<Float> pressureHistory = new ArrayList<Float>();

// --- コピー用（並列処理などで使用） ---
ArrayList<Float> accXHistoryCopy, accYHistoryCopy, accZHistoryCopy;

// --- 起動直後のウォームアップ制御 ---
int warmupTime = 3000;        // ウォームアップ期間（ms単位）
int startTime;                // 起動時刻
boolean isWarmingUp = true;   // ウォームアップ中フラグ

// --- EKF関連グローバル変数 ---
float[] ekfX = new float[9];                        // 状態ベクトル
float[][] P = new float[9][9];                      // 共分散行列
float[][] Q = new float[9][9];                      // プロセスノイズ
float[][] R = new float[9][9];                      // 観測ノイズ
float measuredGyroX, measuredGyroY, measuredGyroZ;  // ジャイロ観測
float measuredDistance;                             // ToF距離観測
float prevX = 0, prevY = 0, prevZ = 0;              // EKF推定の前フレーム位置

// --- 視点操作用変数 ---
float rotX = radians(-30);   // X軸 -30度 初期値
float rotY = radians(-60);   // Y軸 -60度 初期値
float zoom = 620;            // Z軸方向の初期位置
float prevMouseX, prevMouseY;
boolean isDragging = false;
float camOffsetX = 0;
float camOffsetY = 0;
boolean isPanning = false;

void setup() {
  // --- ウィンドウ初期化 ---
  size(800, 800, P3D);
  myPort = new Serial(this, Serial.list()[0], 115200);
  myPort.bufferUntil('\n');
  background(255);
  textSize(14);

  // --- ログファイル生成 ---
  String fileName = "motion_log_" + year() + nf(month(), 2) + nf(day(), 2) + "_" + nf(hour(), 2) + nf(minute(), 2) + ".csv";
  logWriter = createWriter(fileName);
  logWriter.println("time,posX,posY,posZ,theta,stepCount,totalDistance,climbDirection,climbUpCount,climbDownCount,zupt,accX,accY,accZ,pressure,arduinoTime,delay");
  
  // --- Minim初期化 ---
  minim = new Minim(this);
  out = minim.getLineOut();
  beepTone = new BeepTone(1000, 0.5, out.sampleRate());
  out.addSignal(beepTone);
  beepTone.setAmplitude(0);   // 初期状態：ミュート
  
  startTime = millis();       // 起動時刻記録
  
  ekfInit();  // EKFの初期化

}

void draw() {
  // --- ウォームアップ表示（起動後3秒間） ---
  if (isWarmingUp && millis() - startTime < warmupTime) {
    background(255);
    fill(0);
    textAlign(CENTER, CENTER);
    text("Warming Up...", width / 2, height / 2);
    return;
  } else {
    isWarmingUp = false;
  }

  // --- 履歴バッファコピー ---
  synchronized (accXHistory) {
    accXHistoryCopy = new ArrayList<Float>(accXHistory);
  }
  synchronized (accYHistory) {
    accYHistoryCopy = new ArrayList<Float>(accYHistory);
  }
  synchronized (accZHistory) {
    accZHistoryCopy = new ArrayList<Float>(accZHistory);
  }

  // --- 3D描画 ---
  background(255);
  lights();
  translate(width / 2 + camOffsetX, height / 2 + camOffsetY, zoom);
  rotateX(rotX);
  rotateY(rotY);
  rotateZ(0);
  scale(1, 1, -1);
  drawGridBox(20, 100);

  // --- 歩行軌跡 ---
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

  // --- 情報表示 ---
  fill(0);
  hint(DISABLE_DEPTH_TEST);
  camera();
  textAlign(LEFT);
  text("ZUPT: " + zupt, 20, 20);
  text("Step Count: " + stepCount, 20, 40);
  text("Total Distance: " + nf(totalDistance, 1, 2) + " m", 20, 60);
  text("Climb Direction: " + climbDirection, 20, 80);
  text("Climb Up Count: " + climbUpCount, 20, 100);
  text("Climb Down Count: " + climbDownCount, 20, 120);

  drawWave(accZHistoryCopy, color(0, 200, 255), 140, "Z axis (Step Acc)");
  drawWave(accYHistoryCopy, color(0, 255, 100), 200, "Y axis (Climb Acc)");
  drawWave(accXHistoryCopy, color(255, 100, 100), 260, "X axis (Turn Acc)");
  drawCompass(theta, 100, 700, 40);

  hint(ENABLE_DEPTH_TEST);

  detectMotionAndLog();
  
  // --- EKFによる位置推定 ---
  float[] Z = new float[9];
  Z[3] = measuredGyroX;
  Z[4] = measuredGyroY;
  Z[5] = measuredGyroZ;
  Z[6] = posZ;
  Z[7] = ekfX[8];
  Z[8] = measuredDistance;

  float dt = 0.017; // 約60FPS

  ekfPredict(dt);
  ekfUpdate(Z);

  // --- 相対移動として描画座標を更新 ---
  if (stepDetected) {
    float angleRad = radians(theta);  // EKFからの角度
    float dx = stepLength * sin(angleRad);
    float dz = stepLength * cos(angleRad);

    posX += dx;
    posZ += dz;

    trajectory.add(new PVector(posX, posY, posZ));

    prevX = ekfX[0];
    prevY = ekfX[1];
    prevZ = ekfX[2];
  }
  
  // --- ToF距離によるビープ音制御 ---
  // ウォームアップ中はミュート
  if (isWarmingUp) {
    beepTone.setAmplitude(0);
    isBeeping = false;
    return;
  }

  fill(0);
  textAlign(LEFT);
  if (distance < validDistanceMin || distance > validDistanceMax) {
    text("ToF Distance: OUT OF RANGE", 20, 140);
    beepTone.setAmplitude(0);     // ミュート
    isBeeping = false;
    beepStartTime = millis();     // リセット：連続誤検知防止
  } else {
    text("ToF Distance: " + nf(distance / 10.0, 1, 1) + " cm", 20, 140);

    // 音の間隔：距離に応じて条件分岐
    if (distance <= 200) {
      // ～20cm未満：連続音
      beepTone.setAmplitude(0.5);
      isBeeping = true;
      return;
    }
    
    int interval;
    if (distance <= 500) {          // ～50cm：速く
      interval = 400;
    } else if (distance <= 1000) {  // ～1m：遅く
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

void mousePressed() {
  prevMouseX = mouseX;
  prevMouseY = mouseY;

  // Shiftキー押しながらでパン、それ以外は回転
  isPanning = keyPressed && keyCode == SHIFT;
  isDragging = true;
}

void mouseReleased() {
  isDragging = false;
}

void mouseDragged() {
  if (isDragging) {
    float dx = mouseX - prevMouseX;
    float dy = mouseY - prevMouseY;

    if (isPanning) {
      camOffsetX += dx;
      camOffsetY += dy;
    } else {
      rotY += radians(dx) * 0.5;
      rotX += radians(dy) * 0.5;
    }

    prevMouseX = mouseX;
    prevMouseY = mouseY;
  }
}

void mouseWheel(processing.event.MouseEvent event) {
  float e = event.getCount();
  zoom -= e * 20;
  zoom = constrain(zoom, 100, 2000);  // 適切な範囲に制限
}

// --- センサ値の受信処理 ---
void serialEvent(Serial myPort) {
  // Arduinoからのセンサデータを受信・解析（パース）
  String inData = trim(myPort.readStringUntil('\n'));
  if (inData == null || !inData.startsWith("A")) return;
  String[] vals = split(inData, ',');
  if (vals.length < 13) return;

  // 以下を取得
  try {
    float accX = float(vals[1]);
    float accY = float(vals[2]);
    float accZ = float(vals[3]);
    accMag = float(vals[7]);
    magYaw = float(vals[9]);
    distance = int(vals[10]);
    pressure = float(vals[11]);
    arduinoSentTime = int(float(vals[12]));  // Arduino送信時刻
    int receiveTime = millis();              // Processing受信時刻
    systemDelay = receiveTime - arduinoSentTime;

    measuredGyroX = float(vals[7]);
    measuredGyroY = float(vals[8]);
    measuredGyroZ = float(vals[9]);
    measuredDistance = int(vals[12]);        // mm単位

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
    synchronized (pressureHistory) {
      if (pressureHistory.size() > MAX_HISTORY) pressureHistory.remove(0);
      pressureHistory.add(pressure);
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

// --- 歩行・昇降検出とログ出力 ---
void detectMotionAndLog() {
  if (isWarmingUp) return;    // ウォームアップ中は処理停止
  
  climbDirection = "Stable";  // デフォルト状態は常にStable

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

    if (deltaZ > 1.4) {
      stepLength = estimateStepLength(accZHistoryCopy, deltaZ);
      float dz = stepLength * cos(radians(theta));
      float dx = stepLength * sin(radians(theta));
      posZ += dz;
      posX += dx;
      totalDistance += stepLength;
      stepCount++;
      
    // --- 歩行検出フラグを立てる ---
    stepDetected = true;
      
      println("Step Detected: " + stepCount + " (" + nf(stepLength, 1, 2) + " m)");
      
      stepCooldown = 20;
    } else {
    stepDetected = false;  // 検出されなかったらfalseに
    }
  } else if (stepCooldown > 0) {
    stepCooldown--;
    stepDetected = false;  // クールダウン中は検出なし
  }

  // --- 昇降検出：Y軸 ---
  if (!zupt && climbCooldown == 0 && accYHistoryCopy.size() >= 3) {
    float prevY = accYHistoryCopy.get(accYHistoryCopy.size() - 2);
    float currentY = accYHistoryCopy.get(accYHistoryCopy.size() - 1);
    float deltaY = currentY - prevY;
    
    float prevP = pressureHistory.get(pressureHistory.size() - 2);
    float currentP = pressureHistory.get(pressureHistory.size() - 1);
    float deltaPressure = currentP - prevP;

    float prevPrevY = accYHistoryCopy.get(accYHistoryCopy.size() - 3);
    float prevDeltaY = prevY - prevPrevY;

    float threshold = 0.6;  // 昇降判定の変化しきい値

    //println("deltaY: " + nf(deltaY, 1, 3) + " / prevDeltaY: " + nf(prevDeltaY, 1, 3));
    println("prevP: " + nf(prevP, 1, 3) + " / currentP: " + nf(currentP, 1, 3));
    println("deltaPressure: " + nf(deltaPressure, 1, 3));

    float stepHeight = 1.0;
    if (accYHistoryCopy.size() >= 64) {
      float[] fftInput = new float[64];
      for (int i = 0; i < 64; i++) {
        fftInput[i] = accYHistoryCopy.get(accYHistoryCopy.size() - 64 + i);
      }
      float freq = estimateFrequency(fftInput, 30.0);
      stepHeight = constrain(map(freq, 0.5, 3.0, 0.05, 0.2), 0.05, 0.2);
    }

    // --- 判定処理 ---
    if (abs(deltaY) > threshold) {
      if (prevDeltaY < -0.2) {
        climbDirection = "Climbing";
        posY -= stepHeight;
        climbUpCount++;
        println("Climb Detected ↑ (" + nf(stepHeight, 1, 3) + " m)");
        println("deltaY: " + nf(deltaY, 1, 3) + " / prevDeltaY: " + nf(prevDeltaY, 1, 3));
        println("deltaPressure: " + nf(deltaPressure, 1, 3));
      } else if (prevDeltaY > -0.2) {
        climbDirection = "Descending";
        posY += stepHeight;
        climbDownCount++;
        println("Climb Detected ↓ (" + nf(stepHeight, 1, 3) + " m)");
        println("deltaY: " + nf(deltaY, 1, 3) + " / prevDeltaY: " + nf(prevDeltaY, 1, 3));
        println("deltaPressure: " + nf(deltaPressure, 1, 3));
      } else {
        climbDirection = "Stable";
      }
      climbCooldown = 50;
      trajectory.add(new PVector(posX, posY, posZ));
    } else {
      climbDirection = "Stable";
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
                    climbUpCount + "," +
                    climbDownCount + "," +
                    zupt + "," +
                    nf(latestX, 1, 3) + "," +
                    nf(latestY, 1, 3) + "," +
                    nf(latestZ, 1, 3) + "," +
                    nf(pressure, 1, 2) + "," +
                    arduinoSentTime + "," +
                    systemDelay);
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

// --- 歩行周期に基づく歩幅の推定 ---
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
  text(nf(angleDeg, 1, 2) + "°", 0, 0);
  
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

// --- EKF 初期化 ---
void ekfInit(){
  for(int i=0;i<9;i++){
    ekfX[i]=0; // 修正: 状態ベクトル名を ekfX に統一
    for(int j=0;j<9;j++){
      P[i][j]=(i==j)?1.0:0.0;
      Q[i][j]=(i==j)?0.01:0.0;
      R[i][j]=(i==j)?0.1:0.0;
    }
  }
}

// --- 状態遷移関数（非線形） ---
float[] f(float[] X, float dt){
  float[] Xnew = X.clone();
  Xnew[0]+=X[3]*dt;
  Xnew[1]+=X[4]*dt;
  Xnew[2]+=X[5]*dt;
  Xnew[6]+=measuredGyroX*dt;
  Xnew[7]+=measuredGyroY*dt;
  Xnew[8]+=measuredGyroZ*dt;
  return Xnew;
}

// --- 観測関数（非線形） ---
float[] h(float[] X){
  float[] Zpred = new float[9];
  Zpred[0]=0; Zpred[1]=0; Zpred[2]=0;
  Zpred[3]=measuredGyroX;
  Zpred[4]=measuredGyroY;
  Zpred[5]=measuredGyroZ;
  Zpred[6]=X[2];
  Zpred[7]=X[8];
  Zpred[8]=measuredDistance;
  return Zpred;
}

// --- ヤコビアンF ---
float[][] calcJacobianF(float dt){
  float[][] F = identityMatrix(9);
  F[0][3]=dt; F[1][4]=dt; F[2][5]=dt;
  return F;
}

// --- ヤコビアンH ---
float[][] calcJacobianH(){
  float[][] H = new float[9][9];
  H[3][3]=1; H[4][4]=1; H[5][5]=1;
  H[6][2]=1; H[7][8]=1;
  return H;
}

// --- EKF予測 ---
void ekfPredict(float dt){
  float[][] F = calcJacobianF(dt);
  ekfX = f(ekfX, dt);       // 修正
  P = matrixAdd(matrixMultiply(matrixMultiply(F, P), transpose(F)), Q);
}

// --- EKF更新 ---
void ekfUpdate(float[] Z){
  float[][] H = calcJacobianH();
  float[] Zpred = h(ekfX);  // 修正
  float[] y = new float[9];
  for (int i=0; i<9; i++) y[i] = Z[i] - Zpred[i];

  float[][] S = matrixAdd(matrixMultiply(matrixMultiply(H, P), transpose(H)), R);
  float[][] K = matrixMultiply(matrixMultiply(P, transpose(H)), inverseMatrix(S));

  for (int i=0; i<9; i++)
    for (int j=0; j<9; j++)
      ekfX[i] += K[i][j] * y[j];

  float[][] I = identityMatrix(9);
  P = matrixMultiply(matrixSubtract(I, matrixMultiply(K, H)), P);
}

// --- 行列の掛け算（A × B） ---
float[][] matrixMultiply(float[][] A, float[][] B) {
  int aRows = A.length;
  int aCols = A[0].length;
  int bCols = B[0].length;

  float[][] result = new float[aRows][bCols];

  for (int i = 0; i < aRows; i++) {
    for (int j = 0; j < bCols; j++) {
      result[i][j] = 0;
      for (int k = 0; k < aCols; k++) {
        result[i][j] += A[i][k] * B[k][j];
      }
    }
  }
  return result;
}

// --- 行列の加算（A + B） ---
float[][] matrixAdd(float[][] A, float[][] B) {
  int rows = A.length;
  int cols = A[0].length;
  float[][] result = new float[rows][cols];
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      result[i][j] = A[i][j] + B[i][j];
    }
  }
  return result;
}

// --- 行列の減算（A - B） ---
float[][] matrixSubtract(float[][] A, float[][] B) {
  int rows = A.length;
  int cols = A[0].length;
  float[][] result = new float[rows][cols];
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      result[i][j] = A[i][j] - B[i][j];
    }
  }
  return result;
}

// --- 行列の逆行列を求める（ガウス・ジョルダン法） ---
float[][] inverseMatrix(float[][] A) {
  int n = A.length;
  float[][] augmented = new float[n][2 * n];

  // 拡張行列を作る [A | I]
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      augmented[i][j] = A[i][j];
    }
    augmented[i][i + n] = 1.0;
  }

  // ガウス・ジョルダン消去法
  for (int i = 0; i < n; i++) {
    float pivot = augmented[i][i];
    if (abs(pivot) < 1e-6) {
      println("逆行列エラー：ピボットがゼロです");
      return identityMatrix(n);  // 単位行列で代用
    }
    for (int j = 0; j < 2 * n; j++) {
      augmented[i][j] /= pivot;
    }
    for (int k = 0; k < n; k++) {
      if (k != i) {
        float factor = augmented[k][i];
        for (int j = 0; j < 2 * n; j++) {
          augmented[k][j] -= factor * augmented[i][j];
        }
      }
    }
  }

  // 逆行列部分を抽出
  float[][] inverse = new float[n][n];
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      inverse[i][j] = augmented[i][j + n];
    }
  }

  return inverse;
}

// --- 行列の転置 ---
float[][] transpose(float[][] A) {
  int rows = A.length;
  int cols = A[0].length;
  float[][] result = new float[cols][rows];
  
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      result[j][i] = A[i][j];
    }
  }
  return result;
}

// --- 単位行列（Identity Matrix）生成関数 ---
float[][] identityMatrix(int n) {
  float[][] I = new float[n][n];
  for (int i = 0; i < n; i++) {
    I[i][i] = 1.0;
  }
  return I;
}