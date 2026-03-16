from capture import adcCapThread
from config import numADCSamples, numChirpLoops, WaveLength
from process import RDMapVisualizer
import queue


def main():
    config = {
        "numChirpLoops": numChirpLoops,
        "numTx": 3,
        "numRx": 4,
        "numADCSamples": numADCSamples,
        "WaveLength": WaveLength,
    }
    raw_data_queue = queue.Queue(maxsize=5)
    visualizer = RDMapVisualizer(queue_maxsize=10, radar_config=config)
    visualizer.start()

    cap_thread = adcCapThread(
        threadID=1,
        name="RadarCapture",
        static_ip="192.168.33.30",  # 根据你的实际网络配置修改
        adc_ip="192.168.33.180",
        frame_queue=raw_data_queue,
        data_port=4098,
        config_port=4096,
    )
    cap_thread.start()

    print("开始实时处理，按 'q' 键退出...")

    try:
        while True:
            try:
                raw_frame = raw_data_queue.get(timeout=0.1)
                visualizer.update(raw_frame)
            except queue.Empty:
                continue

    except KeyboardInterrupt:
        print("用户中断")
    finally:
        # 清理资源
        cap_thread.stop()
        cap_thread.join()
        visualizer.stop()
        print("程序结束")


if __name__ == "__main__":
    main()
