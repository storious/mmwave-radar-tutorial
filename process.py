import threading
import queue
import cv2
import numpy as np


class RDMapVisualizer(threading.Thread):
    def __init__(self, radar_config, queue_maxsize=10, target_angle=0):
        super().__init__()
        self.daemon = True
        self.data_queue = queue.Queue(maxsize=queue_maxsize)
        self.is_running = threading.Event()
        self.is_running.set()

        # === [新增] 雷达配置参数 ===
        self.cfg = radar_config
        # 解构配置，方便使用
        self.numChirps = radar_config.get("numChirpLoops", 128)
        self.numTx = radar_config.get("numTx", 1)
        self.numRx = radar_config.get("numRx", 4)
        self.numSamples = radar_config.get("numADCSamples", 256)
        self.wavelength = radar_config.get("WaveLength", 0.004)  # 默认 77GHz 波长

        # === [新增] 初始化背景状态 ===
        # 必须在这里初始化，否则每帧都会重置，背景去除失效
        self.background_cube = np.zeros(
            (self.numChirps, self.numTx, self.numRx, self.numSamples),
            dtype=np.complex64,
        )
        self.alpha = 0.05  # 背景更新速率

        # === [新增] 预计算导向矢量 ===
        # 目标角度 0 度，只需计算一次
        target_angle_rad = np.deg2rad(target_angle)
        num_virtual = self.numTx * self.numRx
        # 假设天线间距为半波长
        virtual_ant_positions = np.arange(num_virtual) * self.wavelength / 2
        self.steering_vector = np.exp(
            1j
            * (2 * np.pi / self.wavelength)
            * virtual_ant_positions
            * np.sin(target_angle_rad)
        )

        # 显示参数
        self.colormap = cv2.COLORMAP_VIRIDIS

    def run(self):
        print("可视化线程已启动...")
        while self.is_running.is_set():
            try:
                # 获取原始 int16 数据
                raw_data = self.data_queue.get(block=True, timeout=0.1)

                # 核心处理流程
                frame = self._process_rd_map(raw_data)

                cv2.imshow("Radar RA/DBF View", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            except queue.Empty:
                continue
            except Exception as e:
                print(f"处理异常: {e}")

        cv2.destroyAllWindows()
        print("可视化线程已停止。")

    def update(self, raw_adc_data):
        """外部接口：压入原始 ADC 数据"""
        if not self.is_running.is_set():
            return
        try:
            self.data_queue.put_nowait(raw_adc_data)
        except queue.Full:
            pass

    def stop(self):
        self.is_running.clear()
        self.join()

    def _process_rd_map(self, adc_frame):
        """
        集成了用户自定义的 RA 处理逻辑
        输入: 原始 int16 ADC 数据 (扁平化)
        输出: OpenCV 可显示的图像
        """
        # --- 2. 核心处理逻辑 (来自 process_frame_to_ra) ---

        # A. 背景去除
        # 使用成员变量 self.background_cube 保持状态
        self.background_cube = (
            self.background_cube * (1 - self.alpha) + adc_frame * self.alpha
        )
        static_removed_data = adc_frame - self.background_cube

        # B. Range FFT (axis=3 对应 Samples 维)
        range_data = np.fft.fft(static_removed_data, axis=3)

        # C. Doppler FFT (axis=0 对应 Chirps 维)
        doppler_data = np.fft.fft(range_data, axis=0)

        # D. 虚拟阵列构建与波束成形
        # Transpose to (Doppler, Range, Tx, Rx) -> (Doppler, Range, VirtualAnt)
        # 原始维度: (Chirps, Tx, Rx, Samples)
        # Doppler FFT 后: (Chirps, Tx, Rx, Samples) -> 我们认为这是
        # 现在需要把 Tx, Rx 压平

        # 按照你的逻辑:
        doppler_data_transposed = np.transpose(doppler_data, (0, 3, 1, 2))
        # Shape: (Chirps, Samples, Tx, Rx)

        virtual_array_data = doppler_data_transposed.reshape(
            self.numChirps, self.numSamples, self.numTx * self.numRx
        )
        # Shape: (Doppler, Range, VirtualAnt)

        # E. 数字波束成形 (使用预计算的 self.steering_vector)
        # 对虚拟天线维度求和 -> 仅保留 0 度方向
        beamformed_signal = np.sum(virtual_array_data * self.steering_vector, axis=2)

        # F. 转换为功率并 Shift
        rd_map_power = np.abs(beamformed_signal)
        rd_map_shifted = np.fft.fftshift(rd_map_power, axes=0)

        # G. 转 dB
        rd_map_db = 20 * np.log10(rd_map_shifted + 1e-9)

        # --- 3. 动态范围归一化 (OpenCV 显示) ---
        # 使用你提供的动态范围计算方法
        vmin_dynamic = np.percentile(rd_map_db, 5)
        vmax_dynamic = np.percentile(rd_map_db, 99.9)

        # 归一化到 0-255
        norm_map = (rd_map_db - vmin_dynamic) / (vmax_dynamic - vmin_dynamic + 1e-6)
        norm_map = np.clip(norm_map, 0, 1) * 255
        norm_map = norm_map.astype(np.uint8)

        # --- 4. 伪彩色映射 ---
        # 此时 norm_map 尺寸为
        # 注意 OpenCV 显示时通常 Width=Range, Height=Doppler
        # 为了符合直观，通常转置一下，让 Range 在 X 轴

        color_map = cv2.applyColorMap(norm_map, self.colormap)

        # 可选：放大显示
        color_map = cv2.resize(
            color_map, (1024, 512), fx=2, fy=2, interpolation=cv2.INTER_NEAREST
        )

        return color_map


# === 使用示例 ===
if __name__ == "__main__":
    import config as cfg
    import utils

    # 模拟配置
    config = {
        "numChirpLoops": 128,
        "numTx": 3,
        "numRx": 4,
        "numADCSamples": 256,
        "WaveLength": cfg.WaveLength,
    }
    bin_path = "adc_data_Raw_0.bin"

    chirps_per_frame = cfg.numChirpLoops * 3
    bytes_per_frame = chirps_per_frame * 4 * cfg.numADCSamples * 4

    raw_bytes = np.fromfile(bin_path, dtype=np.uint8)
    num_frames = len(raw_bytes) // bytes_per_frame
    valid_bytes = num_frames * bytes_per_frame
    raw_bytes = raw_bytes[:valid_bytes]
    # --- reshape for processing ---
    adc_data_5d = utils.read_data(
        raw_bytes=raw_bytes,
        numFrames=num_frames,
        num_chirps=chirps_per_frame,
        num_rx=4,
        num_samples=cfg.numADCSamples,
    ).reshape(num_frames, cfg.numChirpLoops, 3, 4, cfg.numADCSamples)

    visualizer = RDMapVisualizer(radar_config=config)
    visualizer.start()

    # 模拟数据流 (模拟 ADC 采集线程)
    for idx in range(num_frames):
        # 生成模拟噪声
        # raw_sim = (np.random.randn(128 * 1 * 4 * 256 * 2) * 100).astype(np.int16)
        raw_sim = adc_data_5d[idx]
        visualizer.update(raw_sim)
        import time

        time.sleep(0.03)

    visualizer.stop()
