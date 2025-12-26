import matplotlib.pyplot as plt
import matplotlib.patches as patches

def draw_piano(highlight_notes=None):
    if highlight_notes is None:
        highlight_notes = []

    # 全鍵の音名（A0〜C8）
    white_notes = ["C", "D", "E", "F", "G", "A", "B"]
    black_notes = ["C#", "D#", None, "F#", "G#", "A#", None]
    note_names = []

    for octave in range(0, 8):
        for w in white_notes:
            note_names.append(f"{w}{octave}")
    note_names.append("C8")  # 最後のC

    # 図の設定
    fig, ax = plt.subplots(figsize=(18, 2))
    ax.set_xlim(0, len(note_names))
    ax.set_ylim(0, 1)
    ax.axis("off")

    # 白鍵を描画
    white_positions = {}
    x = 0
    for note in note_names:
        if "#" not in note:  # 白鍵
            color = "red" if note in highlight_notes else "white"
            rect = patches.Rectangle((x, 0), 1, 1, edgecolor="black", facecolor=color)
            ax.add_patch(rect)
            white_positions[note] = x
            x += 1

    # 黒鍵を描画
    for octave in range(0, 8):
        for i, name in enumerate(black_notes):
            if name is None:
                continue
            full = f"{name}{octave}"
            # 白鍵の相対位置を求める
            white_index = white_notes[i]
            base = f"{white_index}{octave}"
            if base not in white_positions:
                continue
            x = white_positions[base] + 0.7
            color = "red" if full in highlight_notes else "black"
            rect = patches.Rectangle((x - 0.3, 0.4), 0.6, 0.6, facecolor=color, edgecolor="black", zorder=2)
            ax.add_patch(rect)

    plt.show()


# 🔸 使用例（C4とF#3を強調表示）
draw_piano(["C4", "F#3"])
