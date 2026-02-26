import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("result.csv")
plt.scatter(df['obj1_min'], df['obj2_min'], c='blue',marker='o')
plt.scatter(df['obj1_max'], df['obj2_max'], c='red',marker='x')
plt.xlabel('Obj1')
plt.ylabel('Obj2')
plt.title('Pareto Front')
plt.grid(True)  # 显示网格
plt.show()