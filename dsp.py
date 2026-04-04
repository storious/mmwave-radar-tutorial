import numpy as np
from scipy.ndimage import percentile_filter


class RadarSignalProcessor:
    def __init__(self, config):
        self.cfg = config
        self.numSamples = config["numADCSamples"]
        self.numChirps = config["numChirpLoops"]

        # 预计算窗函数
        self.range_win = np.hamming(self.numSamples).astype(np.float32)
        self.doppler_win = np.hamming(self.numChirps).astype(np.float32)

        # 物理参数
        self.wavelength = config.get("WaveLength", 0.0049)

        # ===========================
        # IWR6843ISK 专用配置
        # ===========================
        # 方位窗函数 (8元虚拟阵列) - 使用 Chebyshev 窗抑制副瓣效果更好
        self.az_win = np.hamming(8).astype(np.float32)
        # 俯仰窗函数 (简化处理，通常只有2-4个有效阵元)
        self.el_win = np.hamming(2).astype(np.float32)

    # ... (remove_dc, range_doppler_fft, os_cfar 保持不变) ...
    def remove_dc(self, data):
        real = np.real(data)
        imag = np.imag(data)
        mr = np.mean(real, axis=-1, keepdims=True)
        mi = np.mean(imag, axis=-1, keepdims=True)
        return (real - mr) + 1j * (imag - mi)

    def range_doppler_fft(self, adc_data):
        adc = adc_data.astype(np.complex64)
        adc = self.remove_dc(adc)
        range_data = np.fft.fft(adc * self.range_win[None, None, None, :], axis=-1)
        range_data = range_data[..., : self.numSamples // 2]
        range_data *= self.doppler_win[:, None, None, None]
        doppler_data = np.fft.fft(range_data, axis=0)
        doppler_data = np.fft.fftshift(doppler_data, axes=0)
        return doppler_data

    def os_cfar(self, rdm, win=6, guard=2, rate=75, threshold_db=2.5):
        kernel_size = 2 * (win + guard) + 1
        noise_level = percentile_filter(
            rdm, percentile=rate, size=kernel_size, mode="reflect"
        )
        linear_threshold = 10 ** (threshold_db / 10.0)
        det_map = rdm > (noise_level * linear_threshold)
        pad = win + guard
        det_map[:pad, :] = 0
        det_map[-pad:, :] = 0
        det_map[:, :pad] = 0
        det_map[:, -pad:] = 0
        return det_map

    # ===========================
    # 核心优化：虚拟阵列重构与角度估计
    # ===========================
    def _reconstruct_virtual_array(self, sig_frame):
        """
        针对 IWR6843ISK (3Tx x 4Rx) 重构虚拟阵列
        Input: sig_frame (3, 4) -> (Tx, Rx)
        Output: az_vector (8,), el_vector (approx 2)

        IWR6843ISK 布局逻辑 (参考):
        - Rx 间距 lambda/2
        - Tx2 与 Rx 阵列平行 (方位)
        - Tx1 与 Rx 阵列平行 (方位) -> 与 Tx2 形成扩展孔径
        - Tx0 在俯仰上有偏移
        """
        # sig_frame 维度假设: (Tx, Rx) = (3, 4)
        # 注意：根据你的数据读取逻辑，可能需要转置。这里假设 chirp 维度已被处理，
        # 传入的是某一个 detection 点对应的所有天线数据。

        # 1. 方位虚拟阵列构建 (8 元 ULA)
        # TI IWR6843ISK 的典型 MIMO 映射:
        # 虚拟阵元 [0-3] 通常由 Tx2 + Rx[0-3] 构成
        # 虚拟阵元 [4-7] 通常由 Tx1 + Rx[0-3] 构成 (物理位置连接在前4个之后)
        # 注意：具体顺序取决于 Chirp 配置，这里使用标准 ISK 默认配置逻辑

        tx2_data = sig_frame[2, :]  # Tx2 对应数据 (假设索引 2)
        tx1_data = sig_frame[1, :]  # Tx1 对应数据 (假设索引 1)

        # 拼接成 8 元方位虚拟阵列
        az_data = np.concatenate([tx2_data, tx1_data])

        # 2. 俯仰向量构建 (利用 Tx0)
        # Tx0 通常有俯仰偏移，我们对比 Tx0 与方位中心的相位差
        # 简化：仅使用 Tx0 本身，或 Tx0 与 Tx1/Tx2 的差分
        tx0_data = sig_frame[0, :]
        # 这里简单取 Tx0 的均值作为俯仰参考，或者构建一个 2 元阵进行比较
        # 实际高度计算往往需要更复杂的 2D 角度估计
        el_data = np.array([np.mean(tx0_data), np.mean(tx1_data)])  # 简化的俯仰对比

        return az_data, el_data

    def zoom_fft_angle(self, sig_frame, zoom_factor=8):
        """
        结合虚拟阵列重构的 Zoom-FFT
        Args:
            sig_frame: 某个点对应的 天线数据 (Tx, Rx)
            zoom_factor: 补零倍数
        """
        try:
            # 1. 虚拟阵列重构
            az_sig, el_sig = self._reconstruct_virtual_array(sig_frame)

            # 2. 方位角计算
            # 加窗
            az_sig_win = az_sig * self.az_win
            # 补零
            n_az = len(az_sig_win)
            az_fft = np.fft.fftshift(np.fft.fft(az_sig_win, n=n_az * zoom_factor))

            # 寻峰
            az_idx = np.argmax(np.abs(az_fft))
            # 物理公式计算
            # sin(theta) = (k / N) * lambda / d. (d=lambda/2 -> 2)
            k_az = (az_idx - n_az * zoom_factor / 2.0) / (n_az * zoom_factor)
            sin_az = k_az * 2.0  # d = lambda/2

            if abs(sin_az) > 1.0:
                sin_az = np.clip(sin_az, -1.0, 1.0)
            az_deg = np.degrees(np.arcsin(sin_az))

            # 3. 俯仰角计算 (精度较低)
            # 使用简单的相位差法或小点数 FFT
            # 这里由于阵元少，FFT 效果一般，可用相位差法替代
            # 但为了统一框架，仍使用 FFT
            el_sig_win = el_sig * self.el_win
            n_el = len(el_sig_win)
            el_fft = np.fft.fftshift(np.fft.fft(el_sig_win, n=n_el * zoom_factor))

            el_idx = np.argmax(np.abs(el_fft))
            k_el = (el_idx - n_el * zoom_factor / 2.0) / (n_el * zoom_factor)

            # 俯仰方向的间距可能不是 lambda/2，需要根据天线板规格书调整
            # IWR6843ISK 的 Tx0-Tx1 俯仰间距通常设计为特定值，这里假设约 1.5 lambda 左右的等效间距
            # 若无精确参数，可假设粗略的几何关系
            sin_el = k_el * 2.0  # 近似值，实际需校准

            if abs(sin_el) > 1.0:
                sin_el = np.clip(sin_el, -1.0, 1.0)
            el_deg = np.degrees(np.arcsin(sin_el))

            # 限制范围
            el_deg = np.clip(el_deg, -15.0, 15.0)  # ISK 板通常只能看上方一定角度

            return az_deg, el_deg

        except Exception:
            return 0.0, 3.0

    def get_point_cloud(self, rdm_data, rdm_mag, targets):
        pc = []
        # rdm_data shape: (Doppler, Tx, Rx, Range) 或者? 需确认输入维度
        # 你的 utils 读取代码 reshape 为 -> (Chirps, 3, 4, ADC)
        # 经 FFT 后: (Doppler, 3, 4, Range) -> sig 是 (3, 4)

        for v_bin, r_bin in targets:
            r = r_bin * self.cfg["range_res"]
            vel = (v_bin - self.numChirps // 2) * self.cfg["vel_res"]
            snr = 20 * np.log10(rdm_mag[v_bin, r_bin] / (np.mean(rdm_mag) + 1e-6))

            # 提取该点的天线数据 (Tx, Rx)
            sig = rdm_data[v_bin, :, :, r_bin]

            # 调用优化后的角度计算
            az, el = self.zoom_fft_angle(sig, zoom_factor=8)

            if np.isnan(az):
                continue

            # 坐标转换
            x = r * np.sin(np.radians(az))
            y = r * np.cos(np.radians(az)) * np.cos(np.radians(el))
            z = r * np.sin(np.radians(el))
            # z = 0.1

            if np.isnan(x):
                continue
            pc.append([x, y, z, vel, snr])

        return np.array(pc) if pc else np.empty((0, 5))
