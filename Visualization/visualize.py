import h5py
import numpy as np
import matplotlib.pyplot as plt

# 文件路径
file_path = "/home/kzzwang/Documents/code/DigitTwin/Data/realman_demo/train_data/record_data_0.hdf5"
f = h5py.File(file_path, 'r')

# 打印结构（可选）
def print_structure(name, obj):
    print(name)
f.visititems(print_structure)

# 访问图像数据
cam_top = f["head_images"]

print("cam_top dtype:", cam_top.dtype)
print("cam_top shape:", cam_top.shape)
print("cam_top[0] shape:", cam_top[0].shape)

# cam_top 已经是 (T, H, W, 3) 的 numpy 数组，不需要再解码
img_top = cam_top[0]  # 直接取一帧

# 显示并保存
plt.figure(figsize=(6, 6))
plt.imshow(img_top)
plt.title("cam_top (frame 200)")
plt.axis("off")
plt.tight_layout()
plt.savefig("/home/kzzwang/Documents/code/DigitTwin/Visualization/pic/cam_top_view.png", dpi=300)
# plt.show()

# 读取 qpos 数据
qpos = np.array(f["joint_positions"])  # (T, D)
print("qpos shape:", qpos.shape)
T, D = qpos.shape

# 生成时间步数组
time_steps = np.arange(T)

# 绘图
fig, axs = plt.subplots(D, 1, figsize=(10, 2 * D), sharex=True)
for i in range(D):
    axs[i].plot(time_steps, qpos[:, i])
    axs[i].set_ylabel(f"qpos[{i}]")
    axs[i].grid(True, linestyle="--", alpha=0.5)

# 横轴标注
axs[-1].set_xlabel("Time step")

# 设置横轴刻度（每隔一定步长标一个）
step_interval = max(1, T // 10)  # 根据轨迹长度自动分配10个刻度
axs[-1].set_xticks(np.arange(0, T, step_interval))
axs[-1].set_xticklabels([str(i) for i in range(0, T, step_interval)])

plt.tight_layout()
plt.savefig("/home/kzzwang/Documents/code/DigitTwin/Visualization/pic/qpos_over_time.png", dpi=300)
# plt.show()