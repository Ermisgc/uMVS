import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
import matplotlib.font_manager as fm
import os

# --- 核心：手动指定中文字体路径 ---
def get_chinese_font():
    # 常见的系统字体路径（按优先级排序）
    potential_fonts = [
        'C:/Windows/Fonts/simhei.ttf',          # Windows 黑体
        'C:/Windows/Fonts/msyh.ttc',            # Windows 微软雅黑
        '/System/Library/Fonts/STHeiti Light.ttc', # macOS 华文细黑
        '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc', # Linux 文泉驿正黑
        '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc' # Linux Noto
    ]
    
    for f in potential_fonts:
        if os.path.exists(f):
            # 返回字体属性对象
            return fm.FontProperties(fname=f)
    
    # 如果都没找到，尝试使用系统默认（最后兜底）
    return fm.FontProperties(family='sans-serif')

# 获取字体属性
my_font = get_chinese_font()

# 1. 准备数据
data = {
    'image_index': np.arange(1, 56),
    'match_count': [1565, 1451, 1370, 1323, 1358, 1425, 1443, 1488, 1482, 1503, 1649, 1813, 1925, 2057, 2027, 1959, 1940, 1769, 1457, 1322, 1153, 1059, 1043, 1023, 1011, 1076, 1057, 1054, 1001, 1010, 964, 936, 903, 851, 850, 840, 891, 931, 879, 777, 764, 767, 805, 750, 778, 771, 843, 998, 1099, 1241, 1325, 1391, 1476, 1591, 1728],
    'mean_error': [1.72186, 1.90969, 1.83906, 2.04276, 2.0407, 1.90908, 1.99998, 2.0491, 1.88033, 1.93879, 2.03978, 1.99938, 1.94458, 2.03524, 1.83902, 1.73496, 1.65971, 1.51892, 1.42692, 1.60875, 1.3749, 1.41782, 1.41599, 1.30415, 1.4911, 1.45095, 1.48885, 1.3725, 1.47853, 1.61306, 1.76779, 1.92184, 1.85407, 1.91619, 2.03994, 2.01267, 2.153, 2.25486, 2.1298, 2.32739, 2.20442, 2.12946, 2.17427, 2.39522, 2.7362, 2.52607, 2.44119, 2.39306, 2.38146, 2.42112, 2.72361, 2.5808, 2.79308, 2.79978, 2.43121],
}
df = pd.DataFrame(data)

# 2. 绘图
sns.set_theme(style="whitegrid")
fig, ax1 = plt.subplots(figsize=(12, 6))

# --- 关键：在所有涉及文字的地方指定 fontproperties ---

color_error = 'tab:red'
ax1.set_xlabel('图像样本编号', fontproperties=my_font, fontsize=12)
ax1.set_ylabel('平均垂直像素误差 (pixel)', fontproperties=my_font, color=color_error, fontsize=12)
line1 = ax1.plot(df['image_index'], df['mean_error'], color=color_error, marker='o', markersize=4, label='平均垂直误差', linewidth=1.5)
ax1.tick_params(axis='y', labelcolor=color_error)
ax1.set_ylim(0, 3.5)

ax2 = ax1.twinx() 
color_match = 'tab:blue'
ax2.set_ylabel('匹配特征点数量', fontproperties=my_font, color=color_match, fontsize=12)
bars = ax2.bar(df['image_index'], df['match_count'], color=color_match, alpha=0.3, label='匹配特征点数量')
ax2.tick_params(axis='y', labelcolor=color_match)
ax2.grid(False)

# 图例处理
lines, labels = ax1.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
# 图例也需要设置字体
leg = ax2.legend(lines + lines2, labels + labels2, loc='upper left', frameon=True)
for text in leg.get_texts():
    text.set_fontproperties(my_font)

# plt.title('55组实验数据的校正误差与匹配规模统计图', fontproperties=my_font, fontsize=14, pad=15)
fig.tight_layout()

plt.show()
