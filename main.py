import matplotlib.pyplot as plt
import numpy as np

# 引入自定义模块
import config as cfg
import utils
from visualizer import RDMapVisualizer

if __name__ == "__main__":
    # 配置字典
    radar_config = {
        "numChirpLoops": cfg.numChirpLoops,
        "numADCSamples": cfg.numADCSamples,
        "range_res": cfg.range_resolution,
        "vel_res": cfg.velocity_resolution,
        "height": cfg.Height,
        "angle": cfg.Angle,
    }

    bin_path = "adc_data_Raw_1.bin"
    chirps_per_frame = cfg.numChirpLoops * 3
    bytes_per_frame = chirps_per_frame * 4 * cfg.numADCSamples * 4

    try:
        raw_bytes = np.fromfile(bin_path, dtype=np.uint8)
        num_frames = len(raw_bytes) // bytes_per_frame
        raw_bytes = raw_bytes[: num_frames * bytes_per_frame]
        adc_data = utils.read_data(
            num_frames, raw_bytes, chirps_per_frame, 4, cfg.numADCSamples
        )
        adc_data = adc_data.reshape(
            num_frames, cfg.numChirpLoops, 3, 4, cfg.numADCSamples
        )
        print(f"✅ 数据加载成功: {num_frames} 帧")
    except Exception as e:
        print(f"❌ 数据读取错误: {e}")
        exit()

    # 启动可视化线程
    vis = RDMapVisualizer(radar_config)
    vis.start()

    # Matplotlib 设置
    plt.ion()
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(projection="3d")
    ax.set_box_aspect([1, 1, 1])

    bones = [
        ["head", "neck"],
        ["neck", "shoulderL"],
        ["neck", "shoulderR"],
        ["shoulderL", "elbowL"],
        ["elbowL", "wristL"],
        ["shoulderR", "elbowR"],
        ["elbowR", "wristR"],
        ["neck", "hipL"],
        ["neck", "hipR"],
        ["hipL", "kneeL"],
        ["kneeL", "ankleL"],
        ["hipR", "kneeR"],
        ["kneeR", "ankleR"],
    ]

    for i in range(num_frames):
        vis.update(adc_data[i])

        pc = vis.point_cloud
        skels_with_id = vis.skeletons

        ax.cla()
        ax.set(xlabel="X(m)", ylabel="Y(m)", zlabel="Z(m)")
        ax.set(xlim=(-6, 6), ylim=(0, 12), zlim=(0, 2.5))

        # 绘制区域边界
        th = np.linspace(-np.deg2rad(30), np.deg2rad(30), 50)
        ax.plot(10 * np.sin(th), 10 * np.cos(th), np.zeros_like(th), "g--", alpha=0.5)

        if len(pc) > 0:
            ax.scatter(*pc[:, :3].T, c="gray", s=5, alpha=0.2)

        for t_id, sk in skels_with_id:
            if not sk:
                continue
            color = vis._get_color(t_id)

            # 绘制关节点
            joint_pts = np.array(list(sk.values()))
            ax.scatter(
                *joint_pts[:, :3].T,
                c=[color],
                s=50,
                marker="o",  # 圆形标记
                linewidths=1,
            )

            # 绘制骨骼
            for p1, p2 in bones:
                if p1 in sk and p2 in sk:
                    pt1, pt2 = sk[p1], sk[p2]
                    ax.plot(
                        [pt1[0], pt2[0]],
                        [pt1[1], pt2[1]],
                        [pt1[2], pt2[2]],
                        color=color,
                        linewidth=2,
                    )

            # 标记 ID
            head_pos = sk["head"]
            ax.text(
                head_pos[0],
                head_pos[1],
                head_pos[2] + 0.2,
                f"ID:{t_id}",
                color=color,
                fontsize=10,
            )

        fig.canvas.flush_events()
        plt.pause(0.1)  # 稍微降低刷新频率以保证稳定

    vis.stop()
    plt.close(fig)
