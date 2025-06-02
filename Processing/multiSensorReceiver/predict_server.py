from flask import Flask, request, jsonify           # Webサーバ構築
import joblib                                       # モデルの保存/読み込み
import numpy as np                                  # 特徴量の配列化
import os                                           # ファイル存在確認
import pandas as pd                                 # CSV読み込み・前処理
from sklearn.ensemble import RandomForestRegressor  # 回帰モデル

# Flaskアプリ初期化とファイルパス
app = Flask(__name__)

MODEL_PATH = "step_length_predictor.pkl"
CSV_PATH = "step_features_latest.csv"  # 必要に応じて変更

# モデルを自動学習またはロード
if os.path.exists(MODEL_PATH):
    print("\nLoading the model...")
    model = joblib.load(MODEL_PATH)
else:
    print("\nModel file not found. Training a new model from CSV...")
    if not os.path.exists(CSV_PATH):
        raise FileNotFoundError("CSV file not found: " + CSV_PATH)

    df = pd.read_csv(CSV_PATH)
    X = df[['period', 'accMax', 'accMin', 'accAmp', 'zuptDuration']]    # 説明変数
    y = df['estimatedLength']                                           # 目的変数

    model = RandomForestRegressor(n_estimators=100, random_state=42)
    model.fit(X, y)                 # 学習実行
    joblib.dump(model, MODEL_PATH)  # モデルを保存
    print("Model trained and saved.\n")

# APIエンドポイント
@app.route('/predict', methods=['POST'])
def predict():
    data = request.get_json()   # JSON形式で特徴量を受信
    features = np.array([[      # 2次元配列に変換(1行分)
        data['period'],
        data['accMax'],
        data['accMin'],
        data['accAmp'],
        data['zuptDuration']
    ]])
    prediction = model.predict(features)[0]
    return jsonify({'stepLength': round(float(prediction), 3)})

# Flaskアプリ実行
if __name__ == '__main__':
    app.run(port=5000)