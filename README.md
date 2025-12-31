# チューナーver.ROS2
![test](https://github.com/RyoGH741/mypkg/actions/workflows/test.yml/badge.svg)

## 概要
このros2パッケージは[チューナーコマンド](https://github.com/RyoGH741/robosys2025.git)をros2のトピック通信を利用して実用的なシステムに発展させたものです。

マイクから周波数を計算し、音名と周波数の差を計算し、描画します。

## 依存ライブラリ
- 実行に以下のpythonライブラリが必要となります。
- sounddevice
- aubio
- numpy
- matplotlib
## 導入・実行方法
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
- 実行すると下のように動きます
![Image](https://github.com/user-attachments/assets/fe160ffb-b455-4eb0-bd6a-d69e864d16a3)

## ノードの説明
### mic_freq_pub
- マイクからの音声を取得し、YIN計算により基音の周波数を求めます
- 音声取得に`sounddevice`、YIN計算に`aubip`を使用しています
### tuner_node
- [チューナーコマンド](https://github.com/RyoGH741/robosys2025.git)と機能は同じです。周波数から音名と基準音との差の周波数を求めます
### draw_piano
- tuner_nodeから取得したデータを基に鍵盤を描画します。該当の音が赤色になります

## トピックの説明
### mic_freq
- データ型はFloat32。マイクからの周波数
### note_info
- データ型はChannelFloat32。以下の内容を含んでいます
- neme = 音名
- values = [基準周波数との差、基準周波数]
## 必要なソフトウェア
- ROS2
- Python
## 動作環境

## テスト環境
- Ubuntu 22.04
- ROS2 Humble
- python 3.13.5

## ライセンス
- このソフトウェアパッケージは GNU General Public License v3.0 のもとで後悔されています。
- © 2025 Ryoichi Sakamaki
