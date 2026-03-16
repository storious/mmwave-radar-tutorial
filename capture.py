import socket
import struct
import threading
import queue
import numpy as np

# 假设 config 文件存在且包含所需变量
from config import numChirpLoops, numRx, numTx, numADCSamples

ADC_PARAMS = {
    "chirp": numChirpLoops,
    "rx": numRx,
    "tx": numTx,
    "samples": numADCSamples,
    "IQ": 2,
    "bytes": 2,
}

# STATIC
MAX_PACKET_SIZE = 4096
BYTES_IN_PACKET = 1456  # 确保此值与设备配置一致

# DYNAMIC
BYTES_IN_FRAME = (
    ADC_PARAMS["chirp"]
    * ADC_PARAMS["rx"]
    * ADC_PARAMS["tx"]
    * ADC_PARAMS["IQ"]
    * ADC_PARAMS["samples"]
    * ADC_PARAMS["bytes"]
)

INT16_IN_FRAME = BYTES_IN_FRAME // 2

class adcCapThread(threading.Thread):
    def __init__(
        self,
        threadID,
        name,
        frame_queue=None,  # [新增] 接收一个队列用于实时输出数据
        static_ip="192.168.33.30",
        adc_ip="192.168.33.180",
        data_port=4098,
        config_port=4096,
        bufferSize=1500,
    ):
        threading.Thread.__init__(self)
        self.whileSign = True
        self.threadID = threadID
        self.name = name
        self.frame_queue = frame_queue  # [新增] 保存队列引用
        
        self.recentCapNum = 0
        self.latestReadNum = 0
        self.nextReadBufferPosition = 0
        self.nextCapBufferPosition = 0
        self.bufferOverWritten = False
        self.bufferSize = bufferSize

        # create configuration and data destination
        self.cfg_dest = (adc_ip, config_port)
        self.cfg_recv = (static_ip, config_port)
        self.data_recv = (static_ip, data_port)

        # create socket
        self.config_socket = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP
        )

        self.data_socket = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP
        )

        # binding
        self.data_socket.bind(self.data_recv)
        self.data_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 2**27)
        self.config_socket.bind(self.cfg_recv)

        # 预分配缓冲区 (用于本地存储/回放)
        self.bufferArray = np.zeros(
            (self.bufferSize, BYTES_IN_FRAME // 2), dtype=np.int16
        )
        self.itemNumArray = np.zeros(self.bufferSize, dtype=np.int32)
        self.lostPacketFlagArray = np.zeros(self.bufferSize, dtype=bool)

    def run(self):
        self._frame_receiver()

    def _frame_receiver(self):
        self.data_socket.settimeout(2)

        recentframe = np.zeros(INT16_IN_FRAME, dtype=np.int16)
        recentframe_collect_count = 0
        expected_byte_count = 0

        print(f"{self.name} 开始接收数据...")

        while self.whileSign:
            try:
                packet_num, byte_count, packet_data = self._read_data_packet()
            except socket.timeout:
                print("接收超时，正在重试...")
                continue
            except Exception as e:
                print(f"Socket错误: {e}")
                continue

            # --- 丢包与同步逻辑 ---
            if expected_byte_count != 0 and byte_count != expected_byte_count:
                print(f"警告: 丢包或不同步! 期望 {expected_byte_count}, 实际 {byte_count}")
                recentframe.fill(0)
                recentframe_collect_count = 0
                after_packet_count = (byte_count + BYTES_IN_PACKET) % BYTES_IN_FRAME

                if after_packet_count < BYTES_IN_PACKET:
                    start_idx = (BYTES_IN_PACKET - after_packet_count) // 2
                    recentframe[0 : after_packet_count // 2] = packet_data[start_idx:]
                    recentframe_collect_count = after_packet_count
                    expected_byte_count = byte_count + BYTES_IN_PACKET
                    continue
                else:
                    expected_byte_count = byte_count + BYTES_IN_PACKET
                    continue

            expected_byte_count = byte_count + BYTES_IN_PACKET

            # --- 组帧逻辑 ---
            packet_len = len(packet_data)
            current_idx = recentframe_collect_count // 2
            end_idx = current_idx + packet_len

            if end_idx <= INT16_IN_FRAME:
                recentframe[current_idx:end_idx] = packet_data
                recentframe_collect_count += BYTES_IN_PACKET
            else:
                # 溢出：当前包跨越了两个帧
                remaining_len = INT16_IN_FRAME - current_idx
                recentframe[current_idx:] = packet_data[:remaining_len]

                # 存储当前帧
                self._store_frame(recentframe)

                # 处理剩余数据
                recentframe.fill(0)
                recentframe[0 : packet_len - remaining_len] = packet_data[remaining_len:]
                recentframe_collect_count = (packet_len - remaining_len) * 2

    def getFrame(self):
        """保留原有接口，用于事后读取本地缓冲区"""
        if self.bufferOverWritten:
            self.nextReadBufferPosition = self.nextCapBufferPosition
            self.bufferOverWritten = False
            return None, -1, False

        if self.nextReadBufferPosition == self.nextCapBufferPosition:
            return None, -2, False

        readframe = self.bufferArray[self.nextReadBufferPosition]
        self.latestReadNum = self.itemNumArray[self.nextReadBufferPosition]
        lostPacketFlag = self.lostPacketFlagArray[self.nextReadBufferPosition]

        self.nextReadBufferPosition = (
            self.nextReadBufferPosition + 1
        ) % self.bufferSize

        return readframe, self.latestReadNum, lostPacketFlag

    def _store_frame(self, recentframe):
        # 1. 写入本地环形缓冲区 (保留原有逻辑)
        self.bufferArray[self.nextCapBufferPosition] = recentframe
        self.itemNumArray[self.nextCapBufferPosition] = self.recentCapNum
        self.lostPacketFlagArray[self.nextCapBufferPosition] = False

        # 更新写指针
        self.nextCapBufferPosition = (self.nextCapBufferPosition + 1) % self.bufferSize
        self.recentCapNum += 1

        if self.nextCapBufferPosition == self.nextReadBufferPosition:
            self.bufferOverWritten = True

        # 2. [新增] 推送到实时队列 (配合可视化线程)
        if self.frame_queue is not None:
            try:
                # 使用 put_nowait 避免阻塞采集线程
                # 这里必须使用 .copy()，因为 recentframe 会在下一轮被修改
                self.frame_queue.put_nowait(recentframe.copy())
            except queue.Full:
                # 队列满了说明处理线程来不及处理，直接丢弃最新帧
                # 这样可以保证采集线程不卡顿，实时性好
                pass

    def _read_data_packet(self):
        """读取并解析数据包"""
        data, addr = self.data_socket.recvfrom(MAX_PACKET_SIZE)

        packet_num = struct.unpack("<I", data[:4])[0]
        byte_count_bytes = data[4:10]
        val = struct.unpack("<Q", byte_count_bytes + b"\x00\x00")[0]
        packet_data = np.frombuffer(data[10:], dtype="<i2")

        return packet_num, val, packet_data

    def stop(self):
        """安全停止线程"""
        self.whileSign = False