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
        self.max_lost = 8

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
        self.max_dist = 2.5  # 关联距离

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
        match, used_det = [], set()
        for i, j in zip(row, col):
            if cost[i, j] < self.max_dist:
                match.append((i, j))
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

    def get_smpl_ready(self):
        out = []
        for t in self.trackers:
            out.append([t.id, *t.get_pos(), *t.get_vel()])
        return np.array(out) if out else np.empty((0, 7))


# ===================== 主类（核心修复区）=====================
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

        self.range_res = 0.15
        self.vel_res = 0.08

        # 【修复1】移除背景滤波Cube，改用频域掩膜，彻底消除拖影

        self.colormap = cv2.COLORMAP_VIRIDIS
        self.point_cloud = np.empty((0, 5))

        self.range_win = np.hamming(self.numSamples).astype(np.float32)
        self.doppler_win = np.hamming(self.numChirps).astype(np.float32)

        self.pc_buffer = []
        self.max_pc_buffer = 1

        self.tracker = MultiTracker()
        self.smpl_input = np.empty((0, 7))

    def _remove_dc_advanced(self, data):
        real = np.real(data)
        imag = np.imag(data)
        mr = np.mean(real, axis=-1, keepdims=True)
        mi = np.mean(imag, axis=-1, keepdims=True)
        return (real - mr) + 1j * (imag - mi)

    def _human_pc_filter(self, pc):
        if len(pc) == 0:
            return pc
        snr_ok = pc[:, 4] > 10.0  # SNR稍微放宽一点
        r = np.linalg.norm(pc[:, :3], axis=1)
        range_ok = (r > 0.5) & (r < 10.0)  # 稍微放宽距离
        vel_ok = np.abs(pc[:, 3]) < 3.0  # 放宽速度限制
        z_ok = (pc[:, 2] > -0.5) & (pc[:, 2] < 2.5)  # 放宽高度
        return pc[snr_ok & range_ok & vel_ok & z_ok]

    # 【修复2】优化聚类参数：增大eps，防止单人分裂
    def _mmmesh_dbscan_cluster(self, pc):
        if len(pc) < 3:
            # 如果点太少，直接当做噪声或者单独的目标
            return [], []  # (检测中心列表), (用于SMPL的点云块列表)

        # 【关键】只使用 XY 进行聚类，忽略 Z (高度)，防止人体被切断
        # eps 设为 1.2 米，允许人体点云在水平面上分散
        cls = DBSCAN(eps=1.2, min_samples=3).fit(pc[:, :2])

        labels = cls.labels_

        det_centers = []  # 给跟踪器用的：每个簇的中心坐标
        smpl_blocks = []  # 给 SMPL 用的：每个簇的所有点

        for lab in np.unique(labels):
            if lab == -1:
                continue  # 丢弃噪声点

            # 取出属于这个簇的所有点
            cluster_points = pc[labels == lab]

            # 1. 计算中心点 (用于跟踪)
            # 使用 SNR 加权质心，更稳定
            weights = cluster_points[:, 4]  # SNR
            center = np.average(cluster_points[:, :3], axis=0, weights=weights)
            vel = np.mean(cluster_points[:, 3])

            det_centers.append([center[0], center[1], center[2], vel])

            # 2. 保留所有点 (用于 SMPL)
            smpl_blocks.append(cluster_points)

        return det_centers, smpl_blocks

    def _cfar(self, rdm, guard=2, win=4, th=3.5):
        det = np.zeros_like(rdm, bool)
        H, W = rdm.shape
        # 简化的CA-CFAR
        for i in range(win + guard, H - (win + guard)):
            for j in range(win + guard, W - (win + guard)):
                # 训练区域
                p = rdm[
                    i - win - guard : i + win + guard + 1,
                    j - win - guard : j + win + guard + 1,
                ]
                # 剔除保护区域
                p = np.delete(p, slice(guard, p.shape[0] - guard), 0)
                p = np.delete(p, slice(guard, p.shape[1] - guard), 1)
                noise = np.mean(p)
                if rdm[i, j] > noise * th:
                    det[i, j] = True
        return det

    def _angle(self, sig):
        try:
            # 角度估算
            az = np.fft.fftshift(np.fft.fft(sig.sum(axis=0), 128))
            el = np.fft.fftshift(np.fft.fft(sig.sum(axis=1), 64))
            a = np.clip(((np.argmax(np.abs(az)) - 64) / 64) * 40, -40, 40)
            e = np.clip(((np.argmax(np.abs(el)) - 32) / 32) * 20, -20, 20)
            return a, e
        except Exception:
            return 0.0, 0.0

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

    def _process_rd_map(self, adc_frame):
        adc = adc_frame.astype(np.complex64)

        # 1. 去直流
        adc = self._remove_dc_advanced(adc)

        # 2. Range FFT
        adc *= self.range_win[None, None, None, :]
        range_data = np.fft.fft(adc, axis=-1)
        range_data = range_data[..., : self.numSamples // 2]  # 取前半部分

        # 3. Doppler FFT
        range_data *= self.doppler_win[:, None, None, None]
        doppler_data = np.fft.fft(range_data, axis=0)
        doppler_data = np.fft.fftshift(doppler_data, axes=0)

        # ============================================================
        # 【核心修复】静态杂波抑制：在多普勒域直接将零速度通道置零
        # 这比时域相减干净得多，不会有拖尾，RD图会非常清晰
        # ============================================================
        center_doppler = self.numChirps // 2
        # 将中间的零速度通道及其附近2个通道置零 (共5个通道)
        doppler_data[center_doppler - 1 : center_doppler + 2, :, :, :] = 0

        # 4. 生成RD图
        rdm_mag = np.sum(np.abs(doppler_data), axis=(1, 2))

        # 5. CFAR 检测
        det_map = self._cfar(rdm_mag)
        targets = np.argwhere(det_map)

        # 6. 点云生成与聚类
        pc = self._get_cloud(doppler_data, rdm_mag, targets)
        pc = self._human_pc_filter(pc)

        det_centers, smpl_blocks = self._mmmesh_dbscan_cluster(pc)

        # 7. 跟踪
        self.tracker.predict_all()
        self.tracker.associate(det_centers)
        final_point_cloud = []
        if len(self.tracker.trackers) > 0 and len(smpl_blocks) > 0:
            # 将当前的簇匹配给对应的 Tracker
            # 这里做简化：认为 smpl_blocks 的顺序和 det_centers 一致
            # associate 之后，trackers 的顺序可能被打乱，需要重新匹配

            # 重新匹配逻辑
            for t in self.tracker.trackers:
                t_pos = t.get_pos()
                # 在 det_centers 中找到最近的簇
                min_dist = 999
                best_block = None
                for i, center in enumerate(det_centers):
                    dist = np.linalg.norm(t_pos - center[:3])
                    if dist < min_dist:
                        min_dist = dist
                        best_block = smpl_blocks[i]

                # 如果找到了匹配的块，且距离合理
                if best_block is not None and min_dist < 2.0:
                    # 给这些点打上 ID 标签
                    # best_block 是 (N, 5)，我们增加一列 ID
                    # 但 SMPL 通常只需要 xyz，所以直接用 best_block
                    final_point_cloud.append(best_block)

                    # 这里的 smpl_input 可以存该簇的点云均值或其他特征
                    # 保持接口兼容，这里依然输出中心点信息
                    self.smpl_input = np.array([[t.id, *t.get_pos(), *t.get_vel()]])

        # 合并所有点云用于可视化
        if final_point_cloud:
            self.point_cloud = np.vstack(final_point_cloud)
        else:
            self.point_cloud = np.empty((0, 5))

        # 8. RD图渲染
        db = 20 * np.log10(rdm_mag + 1e-9)
        # 自动对比度调整
        vmin, vmax = np.percentile(db, [5, 99.5])
        norm = ((db - vmin) / (vmax - vmin + 1e-6)).clip(0, 1) * 255
        color = cv2.applyColorMap(norm.astype(np.uint8), self.colormap)

        # 【修复3】图像放大：使用线性插值代替最近邻，图像更平滑清晰
        color = cv2.resize(color, (1024, 512), interpolation=cv2.INTER_LINEAR)

        return color, self.point_cloud

    def run(self):
        print("✅ 修复版：RD图清晰化 + 静态杂波频域抑制 + 聚类优化")
        while self.is_running.is_set():
            try:
                raw = self.data_queue.get(timeout=0.1)
                frame, pc = self._process_rd_map(raw)
                if len(self.smpl_input) > 0:
                    print(f"🧍 实际跟踪人数:{len(self.smpl_input)}")
                cv2.imshow("Radar RD Map | Fixed", frame)
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


# ===================== 主程序 =====================
if __name__ == "__main__":
    import config as cfg
    import utils
    import time

    config = {
        "numChirpLoops": 128,
        "numTx": 3,
        "numRx": 4,
        "numADCSamples": 256,
        "WaveLength": cfg.WaveLength,
    }

    bin_path = "adc_data_Raw_1.bin"
    chirps_per_frame = cfg.numChirpLoops * 3
    bytes_per_frame = chirps_per_frame * 4 * cfg.numADCSamples * 4

    # 这里假设你已经有了读取逻辑，请确保路径正确
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
        print("请确保当前目录下有adc_data_Raw_1.bin文件，且config配置正确")
        exit()

    vis = RDMapVisualizer(config)
    vis.start()

    plt.ion()
    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(projection="3d")

    for i in range(num_frames):
        vis.update(adc_data[i])
        pc = vis.point_cloud
        smpl = vis.smpl_input

        ax.cla()
        ax.set(xlim=(-5, 5), ylim=(0, 15), zlim=(0, 3))
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        if len(pc) > 0:
            ax.scatter(pc[:, 0], pc[:, 1], pc[:, 2], c=pc[:, 3], cmap="viridis", s=12)
        if len(smpl) > 0:
            ax.scatter(smpl[:, 1], smpl[:, 2], smpl[:, 3], c="red", s=80, marker="*")
        fig.canvas.flush_events()
        time.sleep(0.03)

    vis.stop()
    plt.close(fig)
