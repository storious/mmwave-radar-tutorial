import numpy as np
import matplotlib.pyplot as plt
import os
from utils import images_to_video, read_data
from config import (
    numADCSamples,
    numChirpLoops,
    numRx,
    numTx,
    Periodicity,
    FreqSlope,
    C,
    RampEndTime,
    ADCSampleRate,
    ADCStartTime,
    WaveLength,
)
bin_path = "adc_data_Raw_0.bin"

chirps_per_frame = numChirpLoops * numTx
bytes_per_frame = chirps_per_frame * numRx * numADCSamples * 4

raw_bytes = np.fromfile(bin_path, dtype=np.uint8)
num_frames = len(raw_bytes) // bytes_per_frame
valid_bytes = num_frames * bytes_per_frame
raw_bytes = raw_bytes[:valid_bytes]
# --- reshape for processing ---
adc_data_5d = read_data(
    raw_bytes=raw_bytes,
    numFrames=num_frames,
    num_chirps=chirps_per_frame,
    num_rx=numRx,
    num_samples=numADCSamples,
).reshape(num_frames, numChirpLoops, numTx, numRx, numADCSamples)
print(f"重组后的数据维度 (Frames, Loops, Txs, Rxs, Samples): {adc_data_5d.shape}")

# --- 3. 坐标轴计算 ---

# 计算 Tc
Tc = Periodicity / (numChirpLoops * numTx)

# 距离分辨率
range_resolution = C / (2 * FreqSlope * (RampEndTime - ADCStartTime))
max_range = (ADCSampleRate * C) / (2 * FreqSlope)
range_bins_meters = np.linspace(0, max_range, numADCSamples)

print(f"距离分辨率: {range_resolution * 100:.2f} cm")
print(f"最大探测距离: {max_range:.2f} m")

# 速度轴计算
T_loop = Tc * numTx
max_velocity = WaveLength / (4 * T_loop)
velocity_resolution = (2 * max_velocity) / numChirpLoops
velocity_bins_ms = np.linspace(-max_velocity, max_velocity, numChirpLoops)

# --- 4. MIMO 处理流程 (角度估计前的步骤) ---

output_dir = "radar_ra_map"
os.makedirs(output_dir, exist_ok=True)

print(f"开始保存 {num_frames} 帧图像...")

background_cube = np.zeros(
    (numChirpLoops, numTx, numRx, numADCSamples), dtype=np.complex64
)
# 定义更新系数 alpha (0.01 ~ 0.1 之间)，值越小背景越稳定，值越大适应环境变化越快
alpha = 0.05
# 1. 设定感兴趣的角度 (例如: 0 度 正前方)
target_angle_deg = 0
target_angle_rad = np.deg2rad(target_angle_deg)

d = WaveLength / 2
# 生成虚拟阵列的位置向量
# 第 k 个虚拟天线的位置是 k * d
num_virtual = numTx * numRx
virtual_ant_positions = np.arange(num_virtual) * d

# 计算 0 度时的理论相位差
# exp(-1j * ...) 是为了进行相位补偿（共轭）
steering_vector = np.exp(
    1j * (2 * np.pi / WaveLength) * virtual_ant_positions * np.sin(target_angle_rad)
)

# 此时 steering_vector 形状是 (12,)，包含 12 个复数权重

for frame_idx in range(num_frames):
    # 取第一帧进行处理
    adc_frame = adc_data_5d[
        frame_idx
    ]  # Shape: (128, 3, 4, 256) -> (Loops, Txs, Rxs, Samples)

    # 背景 = 旧背景 * (1-alpha) + 当前帧 * alpha
    background_cube = background_cube * (1 - alpha) + adc_frame * alpha

    static_removed_data = adc_frame - background_cube

    # Step 1: Range FFT
    # 注意：使用重排后的数据
    range_data = np.fft.fft(static_removed_data, axis=3)

    # Step 2: Doppler FFT
    doppler_data = np.fft.fft(range_data, axis=0)

    # Step 3: 构建虚拟阵列
    # 先交换轴
    doppler_data_transposed = np.transpose(doppler_data, (0, 3, 1, 2))

    # Reshape 到虚拟阵列
    # 这里的 (numTxAntennas * numRxAntennas) 现在对应的是正确的空间顺序
    virtual_array_data = doppler_data_transposed.reshape(
        numChirpLoops, numADCSamples, numTx * numRx
    )

    # 数据维度: (Doppler, Range, Virtual_Antennas)
    # virtual_array_data 形状: (128, 256, 12)

    # 3. 执行数字波束成形 (相干累积)
    # 我们需要在 (Doppler, Range) 平面上，对 Virtual_Antennas 维度进行加权求和
    # virtual_array_data: (128, 256, 12)
    # steering_vector: (12,)
    # 利用广播机制：(128, 256, 12) * (12,) -> 对最后一维乘法并求和

    beamformed_signal = np.sum(virtual_array_data * steering_vector, axis=2)
    # beamformed_signal 形状变为: (128, 256) -> (Doppler, Range)

    # 4. 处理多普勒维度
    rd_map_power = np.abs(beamformed_signal)

    # --- 5. 显示结果 ---
    rd_map_shifted = np.fft.fftshift(rd_map_power, axes=0)
    # 5. 转换为 dB
    rd_map_db = 20 * np.log10(rd_map_shifted + 1e-9)

    rd_plot = rd_map_shifted.T

    vmin_dynamic = np.percentile(rd_map_db, 5)  # 低于 5% 的值会被切掉（噪声底）
    vmax_dynamic = np.percentile(rd_map_db, 99.9)  # 保留 99.9% 的能量（突出强目标）

    x_axis = np.linspace(-max_velocity, max_velocity, numChirpLoops)
    y_axis = range_bins_meters

    plt.figure(figsize=(12, 8))
    plt.imshow(
        20 * np.log10(rd_plot + 1e-9),
        aspect="auto",
        cmap="viridis",
        origin="lower",
        extent=(x_axis[0], x_axis[-1], y_axis[0], y_axis[-1]),
        interpolation="bilinear",
        vmax=vmax_dynamic,
        vmin=vmin_dynamic,
    )

    plt.colorbar(label="Magnitude (dB)")
    plt.xlabel("Velocity (m/s)")
    plt.ylabel("Range (meters)")
    plt.title(
        f"MIMO RD Map (Reordered Tx: Tx1->Tx2->Tx3)\nMax Range: {y_axis[-1]:.1f}m, Max Vel: {x_axis[-1]:.1f}m/s"
    )
    plt.savefig(f"{output_dir}/frame_{frame_idx:03d}.png", dpi=100)
    plt.close()

images_to_video(output_dir, "ra-view.mp4", fps=10)
