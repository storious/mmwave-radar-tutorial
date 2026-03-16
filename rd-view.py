import numpy as np
import os
import matplotlib.pyplot as plt
from mmwave.dataloader import DCA1000
from utils import images_to_video

# --- 1. 配置参数 (来自 XML) ---
numADCSamples = 256
numTxAntennas = 3
numRxAntennas = 4
numChirpsLoop = 128  # 每个发射天线的 chirp 数
periodicity = 0.1  # s

bin_path = "adc_data_Raw_0.bin"

# --- 2. 读取与解析数据 ---
chirps_per_frame = numChirpsLoop * numTxAntennas
bytes_per_frame = chirps_per_frame * numRxAntennas * numADCSamples * 4

raw_bytes = np.fromfile(bin_path, dtype=np.uint8)

total_bytes = len(raw_bytes)
numFrames = total_bytes // bytes_per_frame

if numFrames == 0:
    print(
        f"错误: 文件太小 ({total_bytes} bytes)，不足以包含一帧数据 ({bytes_per_frame} bytes)"
    )
    exit()

print(f"检测到的总帧数: {numFrames}")

valid_bytes = numFrames * bytes_per_frame
raw_bytes = raw_bytes[:valid_bytes]

adc_data_int16 = raw_bytes.view(dtype=np.int16).reshape(numFrames, -1)

adc_data = np.apply_along_axis(
    DCA1000.organize,
    1,
    adc_data_int16,
    num_chirps=chirps_per_frame,
    num_rx=numRxAntennas,
    num_samples=numADCSamples,
)

print(f"初步数据维度: {adc_data.shape}")

# --- reshape for processing ---
adc_data_5d = adc_data.reshape(
    numFrames, numChirpsLoop, numTxAntennas, numRxAntennas, numADCSamples
)
print(f"重组后的数据维度 (Frames, Loops, Txs, Rxs, Samples): {adc_data_5d.shape}")

# --- 3. 坐标轴计算 ---
start_freq = 60e9
c = 3e8
adc_sample_rate = 4.4e6
freq_slope = 30.012e12
wavelength = c / start_freq
idle = 7e-6
adc_start_time = 6e-6
ramp_end_time = 65e-6

# 计算 Tc
Tc = periodicity / (numChirpsLoop * numTxAntennas)

# 距离分辨率
range_resolution = c / (2 * freq_slope * (ramp_end_time - adc_start_time))
max_range = (adc_sample_rate * c) / (2 * freq_slope)
range_bins_meters = np.linspace(0, max_range, numADCSamples)

print(f"距离分辨率: {range_resolution * 100:.2f} cm")
print(f"最大探测距离: {max_range:.2f} m")

# 速度轴计算
T_loop = Tc * numTxAntennas
max_velocity = wavelength / (4 * T_loop)
velocity_resolution = (2 * max_velocity) / numChirpsLoop
velocity_bins_ms = np.linspace(-max_velocity, max_velocity, numChirpsLoop)

# --- 4. MIMO 处理流程 (角度估计前的步骤) ---

output_dir = "radar_rd_map"
os.makedirs(output_dir, exist_ok=True)

num_frames = adc_data.shape[0]
print(f"开始保存 {num_frames} 帧图像...")

background_cube = np.zeros((128, 3, 4, 256), dtype=np.complex64)
# 定义更新系数 alpha (0.01 ~ 0.1 之间)，值越小背景越稳定，值越大适应环境变化越快
alpha = 0.05 

for frame_idx in range(num_frames):
    # 取第一帧进行处理
    adc_frame = adc_data_5d[
        frame_idx
    ]  # Shape: (128, 3, 4, 256) -> (Loops, Txs, Rxs, Samples)
    # 此时 adc_frame[:, 0, :, :] 是物理第1个发射的数据 (Tx1)
    #      adc_frame[:, 1, :, :] 是物理第2个发射的数据 (Tx3) <-- 注意顺序
    #      adc_frame[:, 2, :, :] 是物理第3个发射的数据 (Tx2)

    # --- Step 1.5: Tx 顺序重排 (关键修改) ---
    # 目标：将数据从 [Tx1, Tx3, Tx2] 重排为 [Tx1, Tx2, Tx3] 以匹配物理天线阵列顺序
    # 原始索引 (数据流顺序): 0(Tx1), 1(Tx3), 2(Tx2)
    # 目标索引 (物理位置顺序): 0(Tx1), 1(Tx2), 2(Tx3)
    # 映射关系: 目标[1] 应该取自 原始[2] (Tx2), 目标[2] 应该取自 原始[1] (Tx3)

    # reorder_indices 的第 i 个元素表示：新的第 i 个位置，应该去原始数据的哪个索引拿
    # new_index[0] = old_index[0] (Tx1 -> Tx1)
    # new_index[1] = old_index[2] (Tx2 <- Tx2, which is at old 2)
    # new_index[2] = old_index[1] (Tx3 <- Tx3, which is at old 1)
    reorder_indices = [0, 2, 1]

    # 使用 numpy 高级索引对 Tx轴 (axis=1) 进行重排
    adc_frame_reordered = adc_frame[:, reorder_indices, :, :]
    # 现在 adc_frame_reordered 的顺序是 [Tx1, Tx2, Tx3]

    # 背景 = 旧背景 * (1-alpha) + 当前帧 * alpha
    background_cube = background_cube * (1 - alpha) + adc_frame_reordered * alpha

    static_removed_data = adc_frame_reordered - background_cube

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
        numChirpsLoop, numADCSamples, numTxAntennas * numRxAntennas
    )

    # Step 4: 非相干积累
    rd_map_raw = np.sum(np.abs(virtual_array_data), axis=2)
    rd_map_shifted = np.fft.fftshift(rd_map_raw, axes=0)

    rd_map_db = 20 * np.log10(rd_map_shifted + 1e-9)

    vmin_dynamic = np.percentile(rd_map_db, 5)  # 低于 5% 的值会被切掉（噪声底）
    vmax_dynamic = np.percentile(rd_map_db, 99.9)  # 保留 99.9% 的能量（突出强目标）

    # --- 5. 显示结果 ---
    plt.figure(figsize=(12, 8))

    rd_plot = rd_map_shifted[numChirpsLoop // 2 :, :].T

    x_axis = np.linspace(0, max_velocity, numChirpsLoop // 2)
    y_axis = range_bins_meters

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
        f"MIMO RD Map (Reordered Tx: Tx1->Tx3->Tx2)\nMax Range: {y_axis[-1]:.1f}m, Max Vel: {x_axis[-1]:.1f}m/s"
    )

    plt.savefig(f'{output_dir}/frame_{frame_idx:03d}.png', dpi=100)
    plt.close()


images_to_video(output_dir, "rd-view.mp4", fps=10)