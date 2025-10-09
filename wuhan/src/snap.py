import omni.kit.viewport.utility as vu
import carb.settings
import omni.replicator.core as rep
from pxr import UsdGeom

def snap():
    prim_camera_path = '/World/overseas_75_b_v_description/head_link2/camera_link/camera_link_follow'

    vp2 = vu.get_active_viewport()
    #print("viewport  name", vp2.name)
    prim = vp2.stage.GetPrimAtPath(prim_camera_path) 

    for attr in ("title", "width", "height", "camera_path", "render_mode"):
        print(f"{attr:15} : {getattr(vp2, attr, None)}")
        
        
    cam = UsdGeom.Camera(prim)
    print("=== 相机 USD 属性 ===")
    print("is camera ",prim.IsA(UsdGeom.Camera))

    out = carb.settings.get_settings().get("./output") or "/home/digitwin/snap"
    vu.capture_viewport_to_file(vp2, f"{out}/vp_snap.png") 
    print("viewport  snap complete", f"{out}/vp_snap.png")


def repli(vp2):
    stage =  omni.usd.get_context().get_stage()

    w = 1186
    h = 1232
    import time
    if prim.IsA(UsdGeom.Camera):          # key check
        cam = UsdGeom.Camera(prim)
        print("=== Camera USD Attributes ===")
        camera   = rep.get.prim_at_path(vp2.camera_path)
        print("prim exist: ", stage.GetPrimAtPath(vp2.camera_path))
        for name in cam.GetSchemaAttributeNames():
    #        print(f'name : {name}')
            attr = prim.GetAttribute(name)
            if attr.HasAuthoredValue():
                print(f"{name:25} : {attr.Get()}")
        rp     = rep.create.render_product(camera, (w, h ))
        writer = rep.WriterRegistry.get("BasicWriter")
        writer.initialize(output_dir="_single", rgb=True)
        writer.attach([rp])
    #     rep.set_global_log_level("DEBUG")
    #    rep.orchestrator.preview()
        rep.orchestrator.run_until_complete_async(1) 
        time.sleep(0.5)
        print('replicator compelete ')
    else:
        print("Prim is not a UsdGeomCamera; check camera_path or search for a Camera child prim.")