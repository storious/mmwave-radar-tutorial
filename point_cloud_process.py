import threading
import queue
import cv2
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from scipy.optimize import linear_sum_assignment


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
        self.max_lost = 12

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
        self.max_dist = 2.5

    def associate(self, detections):
        if len(detections) == 0:
            for t in self.trackers:
                t.lost_cnt += 1
            self.trackers = [t for t in self.trackers if t.lost_cnt < t.max_lost]
            return []
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
        self.numTx = radar_config["numTx"]
        self.numRx = radar_config["numRx"]
        self.numSamples = radar_config["numADCSamples"]
        self.wavelength = radar_config["WaveLength"]

        self.range_res = radar_config["range_res"]
        self.vel_res = radar_config["vel_res"]

        self.colormap = cv2.COLORMAP_VIRIDIS
        self.point_cloud = np.empty((0, 5))
        self.skeletons = []

        self.range_win = np.hamming(self.numSamples).astype(np.float32)
        self.doppler_win = np.hamming(self.numChirps).astype(np.float32)
        self.tracker = MultiTracker()

        # 骨骼稳控参数
        self.skel_buf = None
        self.alpha = 0.20  # 低通：稳为主，轻微跟动作
        self.GROUND_Z = 0.02  # 强制脚踝贴地面

    def _remove_dc_advanced(self, data):
        real = np.real(data)
        imag = np.imag(data)
        mr = np.mean(real, axis=-1, keepdims=True)
        mi = np.mean(imag, axis=-1, keepdims=True)
        return (real - mr) + 1j * (imag - mi)

    # 【超强点云过滤：干掉浮空伪点、杂点】
    def _human_pc_filter(self, pc):
        if len(pc) == 0:
            return pc
        snr_ok = pc[:, 4] > 12.0  # SNR拉高，弱杂点全删
        r = np.linalg.norm(pc[:, :3], axis=1)
        range_ok = (r > 0.5) & (r < 8.0)
        vel_ok = np.abs(pc[:, 3]) < 2.5
        z_ok = (pc[:, 2] > 0.05) & (pc[:, 2] < 2.0)  # 严格人体高度
        return pc[snr_ok & range_ok & vel_ok & z_ok]

    def _dbscan_cluster(self, pc):
        if len(pc) < 5:
            return [], []
        cls = DBSCAN(eps=1.0, min_samples=4).fit(pc[:, :2])
        labels = cls.labels_
        det_centers = []
        smpl_blocks = []
        for lab in np.unique(labels):
            if lab == -1:
                continue
            cluster_points = pc[labels == lab]
            z_mean = np.mean(cluster_points[:, 2])
            if not (0.2 < z_mean < 1.8):
                continue
            weights = cluster_points[:, 4]
            center = np.average(cluster_points[:, :3], axis=0, weights=weights)
            vel = np.mean(cluster_points[:, 3])
            det_centers.append([center[0], center[1], center[2], vel])
            smpl_blocks.append(cluster_points)
        return det_centers, smpl_blocks

    def _cfar(self, rdm, guard=2, win=4, th=3.8):
        det = np.zeros_like(rdm, bool)
        H, W = rdm.shape
        for i in range(win + guard, H - (win + guard)):
            for j in range(win + guard, W - (win + guard)):
                p = rdm[
                    i - win - guard : i + win + guard + 1,
                    j - win - guard : j + win + guard + 1,
                ]
                p = np.delete(p, slice(guard, -guard), 0)
                p = np.delete(p, slice(guard, -guard), 1)
                noise = np.mean(p)
                if rdm[i, j] > noise * th:
                    det[i, j] = True
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

    # ===================== 加固版骨骼：脚贴地 + 生理约束 + 防浮空 =====================
    def _generate_skeleton(self, cluster_points):
        if len(cluster_points) < 8:
            return self.skel_buf

        z_max = np.max(cluster_points[:, 2])
        z_min = np.min(cluster_points[:, 2])
        h_total = z_max - z_min

        # 人体身高硬卡死，异常直接沿用旧骨骼
        if not (1.2 < h_total < 2.0):
            return self.skel_buf

        # 分层裁切
        head_h = z_max - h_total * 0.05
        neck_h = z_max - h_total * 0.18
        shoulder_h = z_max - h_total * 0.28
        hip_h = z_max - h_total * 0.55

        body_cen = np.mean(cluster_points[:, :3], axis=0)

        # 头部
        head_pt = np.array([body_cen[0], body_cen[1], head_h])
        neck_pt = np.array([body_cen[0], body_cen[1], neck_h])

        # 肩膀固定合理宽度，不乱飘
        shl_pt = np.array([body_cen[0] - 0.22, body_cen[1], shoulder_h])
        shr_pt = np.array([body_cen[0] + 0.22, body_cen[1], shoulder_h])

        # 手臂长度卡死
        el_l = shl_pt + np.array([-0.32, 0, -0.3])
        el_r = shr_pt + np.array([+0.32, 0, -0.3])
        wr_l = el_l + np.array([-0.25, 0, -0.22])
        wr_r = el_r + np.array([+0.25, 0, -0.22])

        # 胯部
        hip_l = np.array([body_cen[0] - 0.18, body_cen[1], hip_h])
        hip_r = np.array([body_cen[0] + 0.18, body_cen[1], hip_h])

        # 膝盖
        kn_l = hip_l + np.array([0, 0, -0.45])
        kn_r = hip_r + np.array([0, 0, -0.45])

        # 【核心】脚踝强制钉死地面，永不浮空
        an_l = np.array([kn_l[0], kn_l[1], self.GROUND_Z])
        an_r = np.array([kn_r[0], kn_r[1], self.GROUND_Z])

        skel_new = {
            "head": head_pt,
            "neck": neck_pt,
            "shoulderL": shl_pt,
            "shoulderR": shr_pt,
            "elbowL": el_l,
            "elbowR": el_r,
            "wristL": wr_l,
            "wristR": wr_r,
            "hipL": hip_l,
            "hipR": hip_r,
            "kneeL": kn_l,
            "kneeR": kn_r,
            "ankleL": an_l,
            "ankleR": an_r,
        }

        # 时序低通防抖，杜绝闪跳
        if self.skel_buf is None:
            self.skel_buf = skel_new
        else:
            for k in skel_new.keys():
                self.skel_buf[k] = (1 - self.alpha) * self.skel_buf[
                    k
                ] + self.alpha * skel_new[k]

        return self.skel_buf

    def _process_rd_map(self, adc_frame):
        adc = adc_frame.astype(np.complex64)
        adc = self._remove_dc_advanced(adc)
        range_data = np.fft.fft(adc * self.range_win[None, None, None, :], axis=-1)[
            ..., : self.numSamples // 2
        ]
        doppler_data = np.fft.fft(
            range_data * self.doppler_win[:, None, None, None], axis=0
        )
        doppler_data = np.fft.fftshift(doppler_data, axes=0)
        rdm_mag = np.sum(np.abs(doppler_data), axis=(1, 2))
        det_map = self._cfar(rdm_mag)
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
            for i, c in enumerate(det_centers):
                d = np.linalg.norm(t_pos - c[:3])
                if d < min_d:
                    min_d = d
                    best_blk = smpl_blocks[i]
            if best_blk is not None and min_d < 1.8:
                final_pc.append(best_blk)
                skel = self._generate_skeleton(best_blk)
                if skel:
                    self.skeletons.append(skel)

        self.point_cloud = np.vstack(final_pc) if final_pc else np.empty((0, 5))

        db = 20 * np.log10(rdm_mag + 1e-9)
        vmin, vmax = np.percentile(db, [5, 99.5])
        norm = ((db - vmin) / (vmax - vmin + 1e-6)).clip(0, 1) * 255
        color = cv2.applyColorMap(norm.astype(np.uint8), self.colormap)
        color = cv2.resize(color, (1024, 512), interpolation=cv2.INTER_LINEAR)
        return color

    def run(self):
        while self.is_running.is_set():
            try:
                frame = self._process_rd_map(self.data_queue.get(timeout=0.1))
                cv2.imshow("Radar RD Map", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            except queue.Empty:
                continue
            except Exception as e:
                print("err:", e)
        cv2.destroyAllWindows()

    def update(self, raw):
        try:
            self.data_queue.put_nowait(raw)
        except Exception:
            pass

    def stop(self):
        self.is_running.clear()
        self.join()


# ===================== 主程序：3D火柴人可视化 =====================
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
        ax.set(xlabel="X (m)", ylabel="Y (m)", zlabel="Z (m)")
        ax.set(xlim=(-4, 4), ylim=(0, 10), zlim=(0, 2.2))
        ax.set_box_aspect([1, 1, 1])

        if len(pc) > 0:
            ax.scatter(*pc[:, :3].T, c="blue", s=5, alpha=0.5)

        for sk in skels:
            if not sk:
                continue
            for p1, p2 in bones:
                if p1 in sk and p2 in sk:
                    ax.plot(
                        [sk[p1][0], sk[p2][0]],
                        [sk[p1][1], sk[p2][1]],
                        [sk[p1][2], sk[p2][2]],
                        "r-",
                        linewidth=2,
                    )
            ax.scatter(sk["head"][0], sk["head"][1], sk["head"][2], c="red", s=25)
            ax.scatter(
                sk["ankleL"][0], sk["ankleL"][1], sk["ankleL"][2], c="green", s=20
            )
            ax.scatter(
                sk["ankleR"][0], sk["ankleR"][1], sk["ankleR"][2], c="green", s=20
            )

        fig.canvas.flush_events()
        plt.pause(0.04)

    vis.stop()
    plt.close(fig)
