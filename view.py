import numpy as np
import matplotlib.pyplot as plt
import os
from mmwave.dataloader import DCA1000
from utils import images_to_video


# --- 1. 配置参数 (来自 XML) ---
# demo test
# numADCSamples = 128      # XML: numAdcSamples
# numTxAntennas = 1        # XML: tx0En = 1
# numRxAntennas = 4        # XML: rx0-3En = 1
# numChirpsPerFrame = 2  # XML: sb1numOfLoops
# periodicity = 0.01 # s

numADCSamples = 256 
numTxAntennas = 3       
numRxAntennas = 4        
numChirpsLoop = 128
periodicity = 100 # ms

# 数据文件路径
# demo test
# bin_path = 'C:\\ti\\mmwave_studio_02_01_01_00\\mmWaveStudio\\PostProc\\adc_data.bin'

bin_path = '1770031467.bin'

# --- 2. 读取与解析数据 ---
# 计算单帧字节数: Chirps * Tx * Rx * Samples * 4 bytes (Complex int16)
bytes_per_frame = numChirpsLoop * numTxAntennas * numRxAntennas * numADCSamples * 4

raw_bytes = np.fromfile(bin_path, dtype=np.uint8)

# 计算实际帧数
total_bytes = len(raw_bytes)
numFrames = total_bytes // bytes_per_frame
# assert(numFrames == 5000)

if numFrames == 0:
    print(f"错误: 文件太小 ({total_bytes} bytes)，不足以包含一帧数据 ({bytes_per_frame} bytes)")
    exit()

print(f"检测到的总帧数: {numFrames}")

# 安全截取整数帧的数据
valid_bytes = numFrames * bytes_per_frame
raw_bytes = raw_bytes[:valid_bytes]

# 转换为 int16 并 reshape
adc_data_int16 = raw_bytes.view(dtype=np.int16).reshape(numFrames, -1)

# 使用 organize 函数解析为复数
adc_data = np.apply_along_axis(
    DCA1000.organize,
    1,
    adc_data_int16,
    num_chirps=numChirpsLoop*numTxAntennas,
    num_rx=numRxAntennas,
    num_samples=numADCSamples
)

print(f"最终数据维度: {adc_data.shape}") 

# --- 3. 距离轴计算 (修正版) ---
c = 3e8                       # 光速
adc_sample_rate = 4.4         # 2 MHz (来自 XML: digOutSampleRate=2000)
# freq_slope = 31.249e6         # MHz/s(来自 XML: freqSlopeConst=31.249 MHz/us)
freq_slope = 60.012e6         # MHz/s(来自 XML: freqSlopeConst=31.249 MHz/us)

chirp_time = 65

# 距离分辨率: c / (2 * B)
range_resolution = c / (2 * freq_slope * chirp_time)

# 最大探测距离: (采样率 * c) / (2 * 斜率)
# 等同于: range_resolution * numADCSamples
max_range = (adc_sample_rate * c) / (2 * freq_slope)

# 生成距离轴
range_bins = np.arange(numADCSamples) * range_resolution

print(f"距离分辨率: {range_resolution*100:.2f} cm")
print(f"最大探测距离: {max_range:.2f} m")


# # 创建保存图片的文件夹
output_dir = 'radar_frames'
os.makedirs(output_dir, exist_ok=True)

# 批量保存每一帧
num_frames = adc_data.shape[0]
print(f"开始保存 {num_frames} 帧图像...")

background_cube = np.zeros((384, 4, 256), dtype=np.complex64)
# 定义更新系数 alpha (0.01 ~ 0.1 之间)，值越小背景越稳定，值越大适应环境变化越快
alpha = 0.05 

for frame_idx in range(num_frames):

    frame_data = adc_data[frame_idx] # (chirps, rx, samples)

    # 1. 更新背景模型 (核心步骤)
    # 背景 = 旧背景 * (1-alpha) + 当前帧 * alpha
    background_cube = background_cube * (1 - alpha) + frame_data * alpha

    static_removed_data = frame_data - background_cube
    
    # Range FFT
    range_fft = np.fft.fft(static_removed_data, axis=-1)

    range_mag = np.abs(range_fft)
    range_profile = np.mean(range_mag, axis=(0, 1))
    range_profile_db = 20 * np.log10(range_profile + 1e-9)
    
    # 生成距离轴
    range_bins = np.arange(len(range_profile)) * range_resolution
    
    # 绘图
    plt.figure(figsize=(10, 8))
    plt.plot(range_bins, range_profile_db, color='blue', linewidth=1.5)
    plt.title(f'Frame {frame_idx:03d} - Range Profile')
    plt.xlabel('Range (m)')
    plt.ylabel('Magnitude (dB)')
    plt.grid(True, alpha=0.3)
    
    # 保存图片
    plt.savefig(f'{output_dir}/frame_{frame_idx:03d}.png', dpi=100)
    plt.close()  # 关闭图形，释放内存

print(f"✓ 完成！图片已保存到 {output_dir} 文件夹")

video_name = 'radar_output.mp4'
images_to_video(output_dir, video_name, fps=10)