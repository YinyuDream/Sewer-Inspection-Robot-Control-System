import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

# 1. 读取 CSV 数据
path = os.path.dirname(os.path.abspath(__file__))
df_exec = pd.read_csv(os.path.join(path, "hil_stress_results.csv"))
df_lat = pd.read_csv(os.path.join(path, "hil_stress_latency.csv"))

exec_times = df_exec["exec_time_ms"]
latencies = df_lat["latency_ms"]

# 2. 创建图表（IEEE风格：清晰，对比度高）
try:
    plt.style.use("seaborn-v0_8-whitegrid")
except:
    try:
        plt.style.use("seaborn-whitegrid")
    except:
        plt.style.use("ggplot")

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 4.5), dpi=300)

# 3. 左图：执行时间核密度图
ax1.hist(
    exec_times,
    bins=60,
    density=True,
    alpha=0.4,
    color="#ff7f0e",
    edgecolor="#ff7f0e",
    linewidth=0.5,
)
exec_times.plot.kde(ax=ax1, color="#ff7f0e", linewidth=2)

ax1.set_title(
    "Motion Control Task Execution Time", fontsize=14, fontweight="bold", pad=15
)
ax1.set_xlabel("Execution Time (ms)", fontsize=12, fontweight="bold")
ax1.set_ylabel("Density", fontsize=12, fontweight="bold")
ax1.grid(True, linestyle="--", alpha=0.5)

# 4. 右图：往返延迟核密度图
ax2.hist(
    latencies,
    bins=60,
    density=True,
    alpha=0.4,
    color="#1f77b4",
    edgecolor="#1f77b4",
    linewidth=0.5,
)
latencies.plot.kde(ax=ax2, color="#1f77b4", linewidth=2)

ax2.set_title(
    "CAN Ping-Pong Round-Trip Latency", fontsize=14, fontweight="bold", pad=15
)
ax2.set_xlabel("Latency (ms)", fontsize=12, fontweight="bold")
ax2.set_ylabel("Density", fontsize=12, fontweight="bold")
ax2.grid(True, linestyle="--", alpha=0.5)

plt.tight_layout()
out = os.path.join(path, "hil_stress_kde.png")
plt.savefig(out, dpi=300)
plt.close()
print(f"\u2705 图片已成功生成: hil_stress_kde.png")

# 5. 终端统计输出
stats1 = exec_times.describe()
stats2 = latencies.describe()
print(
    f"\nMotion Control Exec Time (ms): n={len(exec_times)}, "
    f"mean={stats1['mean']:.2f}, median={stats1['50%']:.2f}, "
    f"min={stats1['min']:.2f}, max={stats1['max']:.2f}, std={stats1['std']:.2f}"
)
print(
    f"CAN Latency (ms):            n={len(latencies)}, "
    f"mean={stats2['mean']:.2f}, median={stats2['50%']:.2f}, "
    f"min={stats2['min']:.2f}, max={stats2['max']:.2f}, std={stats2['std']:.2f}"
)
