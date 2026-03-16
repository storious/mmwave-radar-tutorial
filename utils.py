import cv2
import os
import numpy as np
from mmwave.dataloader import DCA1000


# video generate
def images_to_video(image_folder, video_name, fps=30):
    # 1. 获取并排序图片 (合并过滤和排序)
    images = sorted([img for img in os.listdir(image_folder) if img.endswith(".png")])

    if not images:
        print("未找到图片")
        return

    # 2. 获取尺寸 (直接解包 shape)
    first_img = cv2.imread(os.path.join(image_folder, images[0]))
    if first_img is None:
        print("image not exist")
        return

    h, w = first_img.shape[:2]

    # 3. 创建 VideoWriter (修复了 fourcc 写法，并直接传入参数)
    video = cv2.VideoWriter(video_name, cv2.VideoWriter.fourcc(*"mp4v"), fps, (w, h))

    # 4. 循环保存
    for img_name in images:
        img = cv2.imread(os.path.join(image_folder, img_name))
        if img is not None:
            video.write(img)

    video.release()
    print(f"视频已保存: {video_name}")


def read_data(numFrames, raw_bytes, num_chirps, num_rx, num_samples) -> np.ndarray:

    adc_data_int16 = raw_bytes.view(dtype=np.int16).reshape(numFrames, -1)

    adc_data = np.apply_along_axis(
        DCA1000.organize,
        1,
        adc_data_int16,
        num_chirps,
        num_rx,
        num_samples,
    )

    print(f"初步数据维度: {adc_data.shape}")
    return adc_data


def get_steering_vector(numTx, numRx, WaveLength):
    target_angle_deg = 0
    target_angle_rad = np.deg2rad(target_angle_deg)

    num_virtual = numTx * numRx
    virtual_ant_positions = np.arange(num_virtual) * WaveLength / 2

    return np.exp(
        1j * (2 * np.pi / WaveLength) * virtual_ant_positions * np.sin(target_angle_rad)
    )

def process_frame_to_ra(
    adc_frame, numChirpLoops, numTx, numRx, numADCSamples, steering_vector
):

    static_removed_data = clutter_removal_batch(adc_frame)

    # Step 1: Range FFT
    range_data = np.fft.fft(static_removed_data, axis=3)

    # Step 2: Doppler FFT
    doppler_data = np.fft.fft(range_data, axis=0)

    # Step 3: 构建虚拟阵列
    doppler_data_transposed = np.transpose(doppler_data, (0, 3, 1, 2))

    # Reshape 到虚拟阵列
    # 这里的 (numTxAntennas * numRxAntennas) 现在对应的是正确的空间顺序
    virtual_array_data = doppler_data_transposed.reshape(
        numChirpLoops, numADCSamples, numTx * numRx
    )

    # 数据维度: (Doppler, Range, Virtual_Antennas)
    # virtual_array_data 形状: (128, 256, 12)

    # 3. 执行数字波束成形 (相干累积)
    beamformed_signal = np.sum(virtual_array_data * steering_vector, axis=2)
    # beamformed_signal 形状变为: (128, 256) -> (Doppler, Range)

    # 4. 处理多普勒维度
    rd_map_power = np.abs(beamformed_signal)

    rd_map_shifted = np.fft.fftshift(rd_map_power, axes=0)
    # 5. 转换为 dB
    rd_map_db = 20 * np.log10(rd_map_shifted + 1e-9)

    # rd_plot = (20 * np.log10(rd_map_shifted[numChirpLoops:, :].T + 1e-9),)
    vmin_dynamic = np.percentile(rd_map_db, 5)  # 低于 5% 的值会被切掉（噪声底）
    vmax_dynamic = np.percentile(rd_map_db, 99.9)  # 保留 99.9% 的能量（突出强目标）
    return rd_map_db, vmin_dynamic, vmax_dynamic


def clutter_removal_batch(data_cube, chirp_axis=0):
    """
    对整个数据集进行批量背景去除
    """
    # 1. 计算沿 Chirp 维度的平均值
    mean = np.mean(data_cube, axis=chirp_axis, keepdims=True)
    # 2. 减去背景
    data_clean = data_cube - mean
    return data_clean
