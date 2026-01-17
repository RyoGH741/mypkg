# チューナーver.ROS2
![test](https://github.com/RyoGH741/mypkg/actions/workflows/test.yml/badge.svg)

このROS2パッケージには音声の周波数解析と音楽的処理を行うノードが含まれています。各ノードは他のROS2パッケージやノードと組み合わせて利用できるよう設計されています。

## ノード概要
### mic_freq_pub
- マイクから音声を取得し、YINアルゴリズムで基音の周波数を計算します。
- 結果を`freq`トピックとして配信します。
- 音声取得に`sounddevice`、YINアルゴリズムに`aubio`を使用しています。
### tuner_node
- `freq`トピックから受信した周波数をもとに音名と基準周波数、周波数差を計算します。
- 処理内容は[チューナーコマンド](https://github.com/RyoGH741/robosys2025.git)と同様です。
- 結果を`note_info`トピックとして配信します。
### draw_piano
- `note_info`トピックのメッセージをもとにピアノ鍵盤を描画し、該当する音名を赤色で表示します。
- 周波数差と簡易的なメーターも表示させます。

## トピック仕様
### freq
- データ型 : `std_msgs/msg/Float32`
- 内容 : 周波数(Hz)
### note_info
- データ型 : `sensor_msgs/msg/ChannelFloat32`
- メッセージには以下の内容を含みます。
    - name = 音名(例 : A4, Ds5)
    - values = [基準周波数(Hz)、周波数差(Hz)]

## launchファイルについて
- 以下のコマンドを実行すると、本パッケージの3つのノードを同時に立ち上げることができます。
```
& ros2 launch mypkg mic_to_piano.launch.py
```
- 実行すると、以下のように表示されます。
![Image](https://github.com/user-attachments/assets/fe160ffb-b455-4eb0-bd6a-d69e864d16a3)

## 必要なソフトウェア
- ROS2 (Humble)
- Python

## 依存ライブラリ
実行に以下のpythonライブラリが必要です。
- sounddevice
- aubio
- numpy
- matplotlib

## 動作確認環境
- Ubuntu 22.04
- ROS 2 Humble
- Python 3.13.5
- sounddevice 0.5.3
- aubio 0.4.9
- numpy 2.4.0
- matplotlib 3.10.8

## テスト環境
- Ubuntu 22.04
- テストには以下のコンテナを使用しています。
    - [ryuichiueda/ubuntu22.04-ros2](https://hub.docker.com/r/ryuichiueda/ubuntu22.04-ros2)

## ライセンス
- このソフトウェアパッケージは GNU General Public License v3.0 のもとで公開されています。
- © 2025 Ryoichi Sakamaki
- 本プロジェクトで使用している aubio は Paul Brossier 氏および貢献者によって著作権が保持されており、GNU GPL v3.0 のもとで提供されています。


