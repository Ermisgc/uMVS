import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("result.csv")

fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

ax.scatter(df['obj1_min'], df['obj2_min'], df['obj3_min'], c='blue',marker='o', s=20, alpha = 0.7)
# ax.scatter(df['obj1_max'], df['obj2_max'], df['obj3_max'], c='red',marker='x')
ax.view_init(elev=30, azim=45)
ax.set_xlabel('Obj1')
ax.set_ylabel('Obj2')
ax.set_zlabel('Obj3')
ax.set_title('3D Pareto Front')

plt.grid(True)  # 显示网格
plt.show()