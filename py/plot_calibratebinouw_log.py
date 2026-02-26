import pandas as pd
import matplotlib.pyplot as plt

def plot_optimization_log(csv_file):
    try:
        df = pd.read_csv(csv_file)
    except FileNotFoundError:
        print(f"Error: Could not find {csv_file}")
        return

    # 2. 创建一个包含 3 个子图的画布 (3行1列)
    fig, axes = plt.subplots(3, 1, figsize=(10, 12), sharex=True)
    fig.suptitle('NSGA-II 折射标定收敛图', fontsize=16, fontweight='bold')

    # 设定统一的迭代 X 轴
    iterations = df['Iteration']

    # --- 子图 1: 重投影误差 (E_rep) ---
    axes[0].plot(iterations, df['Min_E_rep'], color='#E63946', linewidth=2, label='Min $E_{rep}$')
    axes[0].plot(iterations, df['Max_E_rep'], color='#F4A261', linewidth=2, label='Max $E_{rep}$')
    axes[0].set_ylabel('重投影误差(px)', fontsize=12)
    axes[0].grid(True, linestyle='--', alpha=0.7)
    axes[0].legend(loc='upper right')
    # 如果初始误差极大，可以开启对数坐标轴看得更清楚：
    axes[0].set_yscale('log') 
    
    # --- 子图 2: 尺度误差 (E_scale) ---
    axes[1].plot(iterations, df['Min_E_scale'], color='#2A9D8F', linewidth=2, label='Min $E_{scale}$')
    axes[1].plot(iterations, df['Max_E_scale'], color='#8D99AE', linewidth=2, label='Max $E_{scale}$')
    axes[1].set_ylabel('尺度误差(mm)', fontsize=12)
    axes[1].grid(True, linestyle='--', alpha=0.7)
    axes[1].legend(loc='upper right')

    # --- 子图 3: 共面误差 (E_planar) ---
    axes[2].plot(iterations, df['Min_E_planar'], color='#457B9D', linewidth=2, label='Min $E_{planar}$')
    axes[2].plot(iterations, df['Max_E_planar'], color='#A8DADC', linewidth=2, label='Max $E_{planar}$')
    axes[2].set_xlabel('迭代次数', fontsize=12)
    axes[2].set_ylabel('共面误差(mm)', fontsize=12)
    axes[2].grid(True, linestyle='--', alpha=0.7)
    axes[2].legend(loc='upper right')

    # 3. 调整布局并显示
    plt.tight_layout(rect=[0, 0, 1, 0.96]) # 为主标题留出空间
    
    # 保存高清图表
    plt.savefig('optimization_convergence.png', dpi=300, bbox_inches='tight')
    print("Plot saved as 'optimization_convergence.png'")
    
    # 弹出窗口显示
    plt.show()

if __name__ == "__main__":
    log_path = "calibrateBinoUW_log.csv" 
    plot_optimization_log(log_path)
