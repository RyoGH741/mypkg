import matplotlib.pyplot as plt
import matplotlib.patches as patches

def draw_piano(note ,diff):

    # 全鍵の音名（A0〜C8）
    white_notes = ["C", "D", "E", "F", "G", "A", "H"]
    black_notes = ["Ds", "Es", "Gs", "As", "B"]
    white_note_names = []
    black_note_names = []

    for octave in range(0, 8):
        for w in white_notes:
            white_note_names.append(f"{w}{octave}")
    white_note_names.append("C8")  # 最後のC

    for octave in range(0, 8):
        for w in black_notes:
            black_note_names.append(f"{w}{octave}")
    
    # 図の設定
    fig, ax = plt.subplots(figsize=(12, 3))
    ax.set_xlim(5, 57)
    ax.set_ylim(-1.4, 1.6)
    ax.axis("off")

    # 白鍵を描画
    x = 0
    for i in white_note_names:
        color = "red" if i in note else "white"
        rect = patches.Rectangle((x, 0), 1, 1, edgecolor="black", facecolor=color)
        ax.add_patch(rect)
        x += 1

    # 黒鍵を描画
    x2 = [0,1,3,4,5]
    x2_all = [x + 7 * n for n in range(8) for x in x2]  
    for i, name in enumerate(black_note_names):
        color = "red" if name in note else "black"
        rect = patches.Rectangle((x2_all[i] + 0.5, 0.4), 0.8, 0.6, facecolor=color, edgecolor="black", zorder=2)
        ax.add_patch(rect)

    ax.text(28, 1.3, f"{note}", ha="center", va="center", fontsize=16, color="black", fontweight="bold")
    ax.text(28, -0.5, f"{diff}Hz", ha="center", va="center", fontsize=16, color="black", fontweight="bold")

    # ==== 簡易メーター ====
    if diff < 0:
        meter_text = "- |  0    +"
        label = "low"
    elif diff > 0:
        meter_text = "-    0  | +"
        label = "high"
    else:
        meter_text = "-   |0|   +"
        label = "just"

    ax.text(28, -0.8, meter_text, ha="center", va="center", fontsize=14, color="black", family="monospace")
    ax.text(28, -1.2, f"{label}", ha="center", va="center", fontsize=12, color="black")

    plt.show()

