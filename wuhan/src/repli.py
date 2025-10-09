import omni.replicator.core as rep
import omni.kit.viewport.utility as vu
import carb.settings
import os
import time

# 1. 要旋转的已有 Prim（换成你的）
prim_path = "/World/PaperBox_02"   # 例
prim_camera_path = '/World/overseas_75_b_v_description/head_link2/camera_link/camera_link_follow'
stage = omni.usd.get_context().get_stage()
if not stage.GetPrimAtPath(prim_path).IsValid():
    raise RuntimeError("Prim is not exist")

# 2. 用 Replicator 包装已有 Prim
obj = rep.get.prim_at_path(prim_path)
camera = rep.get.prim_at_path(prim_camera_path)

# 3. 相机 & 渲染产品
rp  = rep.create.render_product(prim_camera_path, (640, 480))

# 4. 强制目录
root = os.path.expanduser("~/pics/rotate")
carb.settings.get_settings().set_string("/omni/replicator/backends/disk/root_dir", root)

i = 0
# 5. 每帧 +5°（共 72 帧 → 360°）
while True:
    with rep.trigger.on_frame():
        with obj: # 对已有 prim 生效
            i = i + 1
            rep.modify.pose(rotation=(i * 5, 0, 0))   # 沿 X 轴每次 +5°
            # 阻塞单帧 + 立即写盘
            # rep.orchestrator.run_until_complete_async(1)
            print(f"frame {i:02d} finished angle={i*5}°")
    time.sleep(0.5)

print(">>> all finished →", os.path.join(root, "_single"))