# チューナーver.ROS2
![test](https://github.com/RyoGH741/mypkg/actions/workflows/test.yml/badge.svg)

## 概要
このROS2パッケージは[チューナーコマンド](https://github.com/RyoGH741/robosys2025.git)をROS2のトピック通信を用いて発展・可視化させたシステムです。

マイク入力からリアルタイムで周波数を解析し、音名と基準音との差を計算してピアノ鍵盤上に表示します。

## 必要なソフトウェア
- ROS2 (Humble)
- Python

## 依存ライブラリ
- 実行に以下のpythonライブラリが必要です。
- sounddevice
- aubio
- numpy
- matplotlib
## インストール・実行方法
```
$ cd ~/ros2_ws/src
$ git clone https://github.com/RyoGH741/mypkg.git
```
```
$ cd ~/ro2_ws
$ git colcon build 
$ source $dir/ros2_ws/install/setup.bash
& ros2 launch mypkg mic_to_piano.launch.py
```
- 実行すると,以下のようにピアノ鍵盤が描画され、検出された音が赤色で表示されます。鍵盤下は周波数差(diff)を示す簡易メーターです。
![Image](https://github.com/user-attachments/assets/fe160ffb-b455-4eb0-bd6a-d69e864d16a3)

## ノード概要
### mic_freq_pub
- マイクから音声を取得し、YINアルゴリズムで基音の周波数を計算します。結果を`mic_freq`トピックとして配信します。
- 音声取得に`sounddevice`、YIN計算に`aubio`を使用しています。
### tuner_node
- `mic_freq`トピックから受信した周波数をもとに[チューナーコマンド](https://github.com/RyoGH741/robosys2025.git)と同様の処理を行い、音名と基準音との周波数差を計算します。結果を`note_info`トピックとして配信します。
### draw_piano
- `note_info`トピックから受信したメッセージをもとにピアノ鍵盤を描画し、該当する音を赤色で表示します。さらに、周波数差と簡易的なメーターも表示させます。

## トピック仕様
### mic_freq
- データ型は`std_msgs/msg/Float32`
- 内容はマイクから取得した基音の周波数(Hz)
### note_info
- データ型は`sensor_msgs/msg/ChannelFloat32`
- 以下の内容を含んでいます
- neme = 音名(例 : A4, Ds5)
- values = [周波数差、基準周波数]

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
-[ryuichiueda/ubuntu22.04-ros2](https://hub.docker.com/r/ryuichiueda/ubuntu22.04-ros2)

## ライセンス
- このソフトウェアパッケージは GNU General Public License v3.0 のもとで公開されています。
- © 2025 Ryoichi Sakamaki
