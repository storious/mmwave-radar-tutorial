import cv2
import os
import numpy as np

# video generate
def images_to_video(image_folder, video_name, fps=30):
    # 获取图片路径列表
    images = [img for img in os.listdir(image_folder) if img.endswith(".png")]
    images.sort() # 确保按顺序排列
    
    # 读取第一张图以获取尺寸
    frame = cv2.imread(os.path.join(image_folder, images[0]))
    height, width, layers = frame.shape

    # 定义视频编码器和创建 VideoWriter 对象
    # 'mp4v' 是常用的 MP4 编码器，也可以用 'avc1'
    fourcc = cv2.VideoWriter_fourcc(*'mp4v') 
    video = cv2.VideoWriter(video_name, fourcc, fps, (width, height))

    print(f"正在处理 {len(images)} 张图片...")

    for image in images:
        video.write(cv2.imread(os.path.join(image_folder, image)))

    cv2.destroyAllWindows()
    video.release()
    print(f"视频已保存为: {video_name}")


def clutter_removal_batch(data_cube, chirp_axis=-1):
    """
    对整个数据集进行批量背景去除
    """
    # 1. 计算沿 Chirp 维度的平均值
    mean = np.mean(data_cube, axis=chirp_axis, keepdims=True)
    # 2. 减去背景
    data_clean = data_cube - mean
    return data_clean
