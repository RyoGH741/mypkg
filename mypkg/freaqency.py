import sounddevice as sd
import numpy as np


def callback(indata, frames, time, status):
    if status:
        print(status)

    # モノラルデータ取得
    audio = indata[:, 0]

    shinpuku = np.sqrt(np.mean(audio ** 2))
    print(shinpuku)

frame = 2048     # 1回の解析サンプル数
device = None    # 特定デバイスを使いたい場合は番号を入れる
fs = 44100
with sd.InputStream(channels=1, samplerate=fs, blocksize=frame, callback=callback, device=device):
    while True:
        pass
