import queue
import threading

import cv2
import matplotlib.pyplot as plt
import numpy as np
from scipy.ndimage import uniform_filter
from scipy.optimize import linear_sum_assignment
from sklearn.cluster import DBSCAN


# ===================== 6维EKF跟踪核心 =====================
class EKFTarget:
    def __init__(self, obj_id, xyz, vel):
        self.id = obj_id
        self.x = np.array([xyz[0], xyz[1], xyz[2], vel[0], 0, 0], dtype=np.float32)
        self.P = np.eye(6, dtype=np.float32) * 2.0
        self.dt = 0.03
        self.F = np.array(
            [
                [1, 0, 0, self.dt, 0, 0],
                [0, 1, 0, 0, self.dt, 0],
                [0, 0, 1, 0, 0, self.dt],
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1],
            ],
            dtype=np.float32,
        )
        self.H = np.array(
            [[1, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0]],
            dtype=np.float32,
        )
        self.Q = np.eye(6) * 0.05
        self.R = np.eye(3) * 0.15
        self.lost_cnt = 0
        self.max_lost = 10  # 缩短超时时间，更快消失
        self.skel_smooth = None

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z):
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P
        self.lost_cnt = 0

    def get_pos(self):
        return self.x[:3]

    def get_vel(self):
        return self.x[3:]


class MultiTracker:
    def __init__(self):
        self.trackers = []
        self.next_id = 1
        self.max_dist = 2.0

    def associate(self, detections):
        if len(detections) == 0:
            for t in self.trackers:
                t.lost_cnt += 1
            self.trackers = [t for t in self.trackers if t.lost_cnt < t.max_lost]
            return
        n_trk, n_det = len(self.trackers), len(detections)
        cost = np.zeros((n_trk, n_det))
        for i, t in enumerate(self.trackers):
            for j, d in enumerate(detections):
                cost[i, j] = np.linalg.norm(t.get_pos() - d[:3])
        row, col = linear_sum_assignment(cost)
        match, used_det = set(), set()
        for i, j in zip(row, col):
            if cost[i, j] < self.max_dist:
                match.add((i, j))
                used_det.add(j)
        for i, j in match:
            self.trackers[i].update(detections[j][:3])
        for j in range(n_det):
            if j not in used_det:
                d = detections[j]
                self.trackers.append(EKFTarget(self.next_id, d[:3], [d[3], 0, 0]))
                self.next_id += 1
        for i in range(n_trk):
            if i not in [m[0] for m in match]:
                self.trackers[i].lost_cnt += 1
        self.trackers = [t for t in self.trackers if t.lost_cnt < t.max_lost]

    def predict_all(self):
        [t.predict() for t in self.trackers]


class RDMapVisualizer(threading.Thread):
    def __init__(self, radar_config, queue_maxsize=10):
        super().__init__()
        self.daemon = True
        self.data_queue = queue.Queue(maxsize=queue_maxsize)
        self.is_running = threading.Event()
        self.is_running.set()

        self.cfg = radar_config
        self.numChirps = radar_config["numChirpLoops"]
        self.numSamples = radar_config["numADCSamples"]
        self.range_res = radar_config["range_res"]
        self.vel_res = radar_config["vel_res"]

        self.colormap = cv2.COLORMAP_VIRIDIS
        self.point_cloud = np.empty((0, 5))
        self.skeletons = []

        self.range_win = np.hamming(self.numSamples).astype(np.float32)
        self.doppler_win = np.hamming(self.numChirps).astype(np.float32)
        self.tracker = MultiTracker()
        self.STD_HEIGHT = 1.75

    def _remove_dc_advanced(self, data):
        real = np.real(data)
        imag = np.imag(data)
        mr = np.mean(real, axis=-1, keepdims=True)
        mi = np.mean(imag, axis=-1, keepdims=True)
        return (real - mr) + 1j * (imag - mi)

    def _human_pc_filter(self, pc):
        if len(pc) == 0:
            return pc
        snr_ok = pc[:, 4] > 8.0
        r = np.linalg.norm(pc[:, :3], axis=1)
        range_ok = (r > 0.5) & (r < 12.0)
        vel_ok = np.abs(pc[:, 3]) < 2.5
        z_ok = (pc[:, 2] > 0.0) & (pc[:, 2] < 2.5)
        return pc[snr_ok & range_ok & vel_ok & z_ok]

    def _dbscan_cluster(self, pc):
        if len(pc) < 5:
            return [], []
        cls = DBSCAN(eps=1.0, min_samples=4).fit(pc[:, :2])
        labels = cls.labels_
        det_centers, smpl_blocks = [], []
        for lab in np.unique(labels):
            if lab == -1:
                continue
            cluster_points = pc[labels == lab]
            if np.mean(cluster_points[:, 2]) > 2.0:
                continue
            weights = cluster_points[:, 4]
            center = np.average(cluster_points[:, :3], axis=0, weights=weights)
            vel = np.mean(cluster_points[:, 3])
            det_centers.append([center[0], center[1], center[2], vel])
            smpl_blocks.append(cluster_points)
        return det_centers, smpl_blocks

    def _cfar_vectorized(self, rdm, win=4, guard=2, th=3.8):
        kernel_size = 2 * (win + guard) + 1
        local_sum = uniform_filter(rdm, size=kernel_size, mode="reflect")
        noise = (local_sum - rdm) / (kernel_size**2 - 1)
        det = rdm > (noise * th)
        pad = win + guard
        det[:pad, :] = 0
        det[-pad:, :] = 0
        det[:, :pad] = 0
        det[:, -pad:] = 0
        return det

    def _angle(self, sig):
        try:
            az = np.fft.fftshift(np.fft.fft(sig.sum(axis=0), 128))
            el = np.fft.fftshift(np.fft.fft(sig.sum(axis=1), 64))
            a = np.clip(((np.argmax(np.abs(az)) - 64) / 64) * 35, -35, 35)
            e = np.clip(((np.argmax(np.abs(el)) - 32) / 32) * 15, 1, 15)
            return a, e
        except Exception:
            return 0.0, 3.0

    def _get_cloud(self, rdm_data, rdm_mag, targets):
        pc = []
        for v_bin, r_bin in targets:
            r = r_bin * self.range_res
            vel = (v_bin - self.numChirps // 2) * self.vel_res
            snr = 20 * np.log10(rdm_mag[v_bin, r_bin] / (np.mean(rdm_mag) + 1e-6))
            sig = rdm_data[v_bin, :, :, r_bin]
            az, el = self._angle(sig)
            x = r * np.sin(np.radians(az))
            y = r * np.cos(np.radians(az))
            z = abs(r * np.sin(np.radians(el)))
            if np.isnan(x):
                continue
            pc.append([x, y, z, vel, snr])
        return np.array(pc) if pc else np.empty((0, 5))

    def _generate_skeleton(self, cluster_points, tracker_obj):
        # 【逻辑加固1】点数必须大于关节数(14)，否则视为噪点，不画人
        if len(cluster_points) < 15:
            return None

        # 【逻辑加固2】位置必须服从点云分布，而不是 EKF
        # 使用点云的实际质心作为 X, Y
        obs_xy = np.mean(cluster_points[:, :2], axis=0)
        base_x, base_y = obs_xy[0], obs_xy[1]

        # Z轴使用点云的底部
        z_vals = cluster_points[:, 2]
        base_z = np.percentile(z_vals, 10)

        # 固定高度
        H = self.STD_HEIGHT

        skel_new = {
            "head": np.array([base_x, base_y, base_z + H]),
            "neck": np.array([base_x, base_y, base_z + H * 0.85]),
            "shoulderL": np.array([base_x - 0.2, base_y, base_z + H * 0.80]),
            "shoulderR": np.array([base_x + 0.2, base_y, base_z + H * 0.80]),
            "hipL": np.array([base_x - 0.15, base_y, base_z + H * 0.50]),
            "hipR": np.array([base_x + 0.15, base_y, base_z + H * 0.50]),
            "ankleL": np.array([base_x - 0.1, base_y, base_z + 0.02]),
            "ankleR": np.array([base_x + 0.1, base_y, base_z + 0.02]),
        }
        skel_new["elbowL"] = skel_new["shoulderL"] + np.array([-0.05, 0, -0.3])
        skel_new["wristL"] = skel_new["elbowL"] + np.array([-0.05, 0, -0.25])
        skel_new["elbowR"] = skel_new["shoulderR"] + np.array([0.05, 0, -0.3])
        skel_new["wristR"] = skel_new["elbowR"] + np.array([0.05, 0, -0.25])
        skel_new["kneeL"] = skel_new["hipL"] + np.array([0, 0, -0.45])
        skel_new["kneeR"] = skel_new["hipR"] + np.array([0, 0, -0.45])

        # 平滑
        alpha = 0.4  # 加快响应速度，因为位置现在跟随点云
        if tracker_obj.skel_smooth is None:
            tracker_obj.skel_smooth = skel_new
        else:
            for k in skel_new.keys():
                tracker_obj.skel_smooth[k] = (1 - alpha) * tracker_obj.skel_smooth[
                    k
                ] + alpha * skel_new[k]

        return tracker_obj.skel_smooth

    def _process_rd_map(self, adc_frame):
        adc = adc_frame.astype(np.complex64)
        adc = self._remove_dc_advanced(adc)

        range_data = np.fft.fft(adc * self.range_win[None, None, None, :], axis=-1)[
            ..., : self.numSamples // 2
        ]
        range_data *= self.doppler_win[:, None, None, None]
        doppler_data = np.fft.fft(range_data, axis=0)
        doppler_data = np.fft.fftshift(doppler_data, axes=0)

        rdm_mag = np.sum(np.abs(doppler_data), axis=(1, 2))
        det_map = self._cfar_vectorized(rdm_mag)
        targets = np.argwhere(det_map)

        pc = self._get_cloud(doppler_data, rdm_mag, targets)
        pc = self._human_pc_filter(pc)

        det_centers, smpl_blocks = self._dbscan_cluster(pc)

        self.tracker.predict_all()
        self.tracker.associate(det_centers)

        final_pc = []
        self.skeletons = []

        for t in self.tracker.trackers:
            t_pos = t.get_pos()
            best_blk = None
            min_d = 999
            best_idx = -1

            # 寻找最近的簇
            for i, c in enumerate(det_centers):
                d = np.linalg.norm(t_pos - c[:3])
                if d < min_d:
                    min_d = d
                    best_blk = smpl_blocks[i]

            # 【核心逻辑修复】
            # 1. 必须匹配到簇 (min_d < 1.8)
            # 2. 匹配到簇后，是否画骨架完全取决于 _generate_skeleton 内部的点数判断
            if best_blk is not None and min_d < 1.8:
                final_pc.append(best_blk)
                skel = self._generate_skeleton(best_blk, t)
                if skel:
                    self.skeletons.append(skel)
                else:
                    # 如果点数不够画骨架，清空该 ID 的缓存，防止下一帧画出旧位置
                    t.skel_smooth = None
            else:
                # 如果没匹配到点云，清空骨架缓存，杜绝“幽灵人”
                t.skel_smooth = None

        self.point_cloud = np.vstack(final_pc) if final_pc else np.empty((0, 5))

        db = 20 * np.log10(rdm_mag + 1e-9)
        vmin, vmax = np.percentile(db, [5, 99.5])
        norm = ((db - vmin) / (vmax - vmin + 1e-6)).clip(0, 1) * 255
        color = cv2.applyColorMap(norm.astype(np.uint8), self.colormap)
        return cv2.resize(color, (1024, 512), interpolation=cv2.INTER_LINEAR)

    def run(self):
        print("✅ 严格逻辑版: 点云中心定位 + 密度过滤 + 清除幽灵")
        while self.is_running.is_set():
            try:
                frame = self._process_rd_map(self.data_queue.get(timeout=0.1))
                cv2.imshow("Radar RD Map", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            except queue.Empty:
                continue
        cv2.destroyAllWindows()

    def update(self, raw):
        try:
            self.data_queue.put_nowait(raw)
        except Exception:
            pass

    def stop(self):
        self.is_running.clear()
        self.join()


# ===================== 主程序 =====================
if __name__ == "__main__":
    import config as cfg
    import utils

    config = {
        "numChirpLoops": cfg.numChirpLoops,
        "numTx": cfg.numTx,
        "numRx": cfg.numRx,
        "numADCSamples": cfg.numADCSamples,
        "WaveLength": cfg.WaveLength,
        "range_res": cfg.range_resolution,
        "vel_res": cfg.velocity_resolution,
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
    except Exception as e:
        print(f"数据读取错误: {e}")
        exit()

    vis = RDMapVisualizer(config)
    vis.start()

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
        skels = vis.skeletons

        ax.cla()
        ax.set(xlabel="X(m)", ylabel="Y(m)", zlabel="Z(m)")
        ax.set(xlim=(-4, 4), ylim=(0, 10), zlim=(0, 2.2))
        ax.set_box_aspect([1, 1, 1])

        # 画点云
        if len(pc) > 0:
            ax.scatter(*pc[:, :3].T, c="blue", s=5, alpha=0.5)

        # 画骨架
        for sk in skels:
            if not sk:
                continue
            joint_pts = np.array(list(sk.values()))
            ax.scatter(
                *joint_pts[:, :3].T,
                c="red",
                s=20,
                zorder=5,
            )

            for p1, p2 in bones:
                if p1 in sk and p2 in sk:
                    pt1, pt2 = sk[p1], sk[p2]
                    ax.plot(
                        [pt1[0], pt2[0]],
                        [pt1[1], pt2[1]],
                        [pt1[2], pt2[2]],
                        "r-",
                        linewidth=2,
                    )

        fig.canvas.flush_events()
        plt.pause(0.001)

    vis.stop()
    plt.close(fig)
