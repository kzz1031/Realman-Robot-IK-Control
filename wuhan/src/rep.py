import omni.replicator.core as rep

prim_path = "/World/PaperBox_02"   # 例
prim_camera_path = '/World/overseas_75_b_v_description/head_link2/camera_link/camera_link_follow'

with rep.new_layer():

    # Add Default Light
    # distance_light = rep.create.light(rotation=(315,0,0), intensity=3000, light_type="distant")

    render_product = rep.create.render_product(prim_camera_path, (2048, 1024))

    def rorate_boxes():
        box = rep.get.prim_at_path(prim_path)
        with box:
            rep.modify.pose(
                rotation=rep.distribution.uniform((-180,0, 0), (180, 0, 0)),
            )

    rep.randomizer.register(rorate_boxes)

    # Setup randomization
    with rep.trigger.on_frame(num_frames=100):
        rep.randomizer.rorate_boxes()
        writer = rep.WriterRegistry.get("BasicWriter")
        writer.initialize(output_dir="_single", rgb=True)
        writer.attach([render_product])