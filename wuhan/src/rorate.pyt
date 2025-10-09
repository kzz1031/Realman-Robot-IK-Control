from pxr import UsdGeom, Gf
prim_path_box = "/World/PaperBox_02"   # 例

stage = omni.usd.get_context().get_stage()

prim  = stage.GetPrimAtPath(prim_path_box)
if not stage.GetPrimAtPath(prim_path_box).IsValid():
    raise RuntimeError("Prim is not exist")
# 1. 读当前 *local* 4×4 矩阵（时间 0）
local_mtx = Gf.Matrix4d()
for op in xform.GetOrderedXformOps():
    val = op.Get(0)                       # 0 = 时间码
    if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
        local_mtx = local_mtx * Gf.Matrix4d().SetTranslate(val)
    elif op.GetOpType() == UsdGeom.XformOp.TypeRotateYXZ:
        local_mtx = local_mtx * Gf.Matrix4d().SetRotate(Gf.Rotation(*val))
    elif op.GetOpType() == UsdGeom.XformOp.TypeScale:
        local_mtx = local_mtx * Gf.Matrix4d().SetScale(val)

# 2. 绕 X +30° 局部旋转
rot30 = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), 30))
new_mtx = local_mtx * rot30   # 右乘：先转 30° 再保持原变换

# 3. 写回 Prim（覆盖原 transform）
xform.SetMatrix(new_mtx)