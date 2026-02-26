import matplotlib.pyplot as plt
import numpy as np

# --- Windows 强效修复乱码方案 ---
# 1. 优先使用微软雅黑，如果没有则使用黑体
plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'Arial Unicode MS'] 
# 2. 必须在设置字体后立即关闭 unicode_minus
plt.rcParams['axes.unicode_minus'] = False 

# 3. 针对对数坐标轴负号的特殊处理：强制使用标准 ASCII 连字符
import matplotlib.ticker as ticker

# 1. 整理数据
methods = ['Eigen精确解法', '2.3.1小节的方法', '2.3.2小节的方法', '牛顿迭代法']
times = [12.9614, 1.120, 1.783, 8.35] 
errors = [5.23e-3, 0.03938, 5.2489e-5, 3.61e-4] 
max_errors = [0.054, 0.222, 9.2146e-5, 0.01] 

# 2. 绘图配置
fig, ax = plt.subplots(figsize=(10, 7), dpi=100)
colors = ['#555555', '#E74C3C', '#2ECC71', '#3498DB'] 
markers = ['o', 's', '*', '^'] 

# 3. 绘制散点和误差棒
for i in range(len(methods)):
    ax.scatter(times[i], errors[i], color=colors[i], marker=markers[i], s=200, 
                label=methods[i], edgecolors='black', zorder=5)
    ax.errorbar(times[i], errors[i], 
                 yerr=[[0], [max_errors[i] - errors[i]]], 
                 fmt='none', ecolor=colors[i], capsize=5, elinewidth=2, alpha=0.6)

# 4. 设置坐标轴为对数刻度
ax.set_yscale('log')

# --- 关键修复点：如果 log 坐标轴的负号还是方框，手动强制格式化 ---
ax.yaxis.set_major_formatter(ticker.LogFormatterSciNotation())

# 5. 修饰与标签
ax.grid(True, which="both", ls="--", alpha=0.5)
ax.set_xlabel('平均执行耗时 (μs)', fontsize=14)
ax.set_ylabel('平均重投影误差 (pixel)', fontsize=14)

# 6. 标注
# ax.annotate('本研究方法：极致效率与高精度', 
#              xy=(1.783, 5.24e-5), 
#              xytext=(3.5, 2e-4),
#              arrowprops=dict(facecolor='black', shrink=0.05, width=1.2, headwidth=8),
#              fontsize=12, color='#1B5E20', fontweight='bold')

# 7. 设置图例和范围
ax.legend(loc='best', fontsize=11, frameon=True)
ax.set_xlim(0, 15)
ax.set_ylim(1e-5, 1)

plt.tight_layout()
plt.show()
