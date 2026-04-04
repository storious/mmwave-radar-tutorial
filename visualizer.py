import colorsys
import queue
import threading

import cv2
import numpy as np

from clustering import AdaptiveClusterer

# 引入其他模块
from dsp import RadarSignalProcessor
from skeleton import SkeletonGenerator
from tracking import MultiTracker, ZoneManager


class RDMapVisualizer(threading.Thread):
    def __init__(self, radar_config):
        super().__init__()
        self.daemon = True
        self.data_queue = queue.Queue(maxsize=10)
        self.is_running = threading.Event()
        self.is_running.set()

        # 初始化各模块
        self.dsp = RadarSignalProcessor(radar_config)
        self.clusterer = AdaptiveClusterer()
        self.zone = ZoneManager(max_range=10.0, fov_deg=60.0)
        self.tracker = MultiTracker(self.zone)
        self.skeleton_gen = SkeletonGenerator()

        self.colormap = cv2.COLORMAP_VIRIDIS
        self.point_cloud = np.empty((0, 5))
        self.skeletons = []

    def _get_color(self, obj_id):
        hue = (obj_id * 0.618033988749895) % 1.0
        return colorsys.hsv_to_rgb(hue, 0.9, 1.0)

    def run(self):
        print("🚀 优化系统启动: OS-CFAR + Zoom-FFT + 自适应聚类")
        while self.is_running.is_set():
            try:
                adc_frame = self.data_queue.get(timeout=0.1)

                # 1. DSP 处理
                doppler_data = self.dsp.range_doppler_fft(adc_frame)
                rdm_mag = np.sum(np.abs(doppler_data), axis=(1, 2))

                # 使用 OS-CFAR
                det_map = self.dsp.os_cfar(rdm_mag, rate=75, threshold_db=2.8)
                targets = np.argwhere(det_map)

                # 获取点云 (含 Zoom-FFT)
                pc = self.dsp.get_point_cloud(doppler_data, rdm_mag, targets)
                pc = self.clusterer.filter_points(pc)

                # 2. 聚类
                det_centers, smpl_blocks = self.clusterer.cluster(pc)

                # 3. 跟踪
                self.tracker.associate(det_centers)

                # 4. 骨架生成
                final_pc = []
                self.skeletons = []
                used_blocks = set()

                # 关联逻辑
                potential_matches = []
                for t in self.tracker.trackers:
                    t_pos = t.get_pos()
                    for i, c in enumerate(det_centers):
                        dist = np.linalg.norm(t_pos - c[:3])
                        if dist < 1.8:
                            potential_matches.append((t.id, i, dist))

                potential_matches.sort(key=lambda x: x[2])

                for t_id, b_idx, dist in potential_matches:
                    if b_idx in used_blocks:
                        continue
                    t_obj = next(
                        (t for t in self.tracker.trackers if t.id == t_id), None
                    )
                    if t_obj:
                        blk = smpl_blocks[b_idx]
                        final_pc.append(blk)
                        skel = self.skeleton_gen.generate(blk, t_obj)
                        if skel:
                            self.skeletons.append((t_id, skel))
                        used_blocks.add(b_idx)

                self.point_cloud = np.vstack(final_pc) if final_pc else np.empty((0, 5))

                # 5. RD 图显示
                db = 20 * np.log10(rdm_mag + 1e-9)
                vmin, vmax = np.percentile(db, [5, 99.5])
                norm = ((db - vmin) / (vmax - vmin + 1e-6)).clip(0, 1) * 255
                rd_img = cv2.applyColorMap(norm.astype(np.uint8), self.colormap)
                cv2.imshow("Radar RD Map", cv2.resize(rd_img, (1024, 512)))
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            except queue.Empty:
                continue
        cv2.destroyAllWindows()

    def update(self, raw):
        try:
            self.data_queue.put_nowait(raw)
        except Exception as e:
            print(f"update failed {e}")
            pass

    def stop(self):
        self.is_running.clear()
        self.join()
