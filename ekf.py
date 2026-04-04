import numpy as np


class EKFTarget:
    """
    优化后的扩展卡尔曼滤波目标类
    状态向量: [x, y, z, vx, vy, vz]
    观测向量: [x, y, z]
    """

    # ===========================
    # 1. 硬编码常量矩阵 (类级别共享，只加载一次)
    # ===========================

    # 状态转移矩阵 F (6x6), dt = 0.03
    # 位置 = 位置 + 速度 * dt
    _F = np.array(
        [
            [1.0, 0.0, 0.0, 0.03, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0, 0.03, 0.0],
            [0.0, 0.0, 1.0, 0.0, 0.0, 0.03],
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float32,
    )

    # 观测矩阵 H (3x6) - 仅观测位置
    _H = np.array(
        [
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        ],
        dtype=np.float32,
    )

    # 过程噪声 Q (6x6)
    _Q = np.diag([0.05, 0.05, 0.05, 0.05, 0.05, 0.05]).astype(np.float32)

    # 观测噪声 R (3x3)
    _R = np.diag([0.15, 0.15, 0.15]).astype(np.float32)

    # 单位矩阵 I (6x6)
    _I = np.eye(6, dtype=np.float32)

    def __init__(self, obj_id, xyz, vel):
        self.id = obj_id

        # 状态向量 x (6,)
        self.x = np.array([xyz[0], xyz[1], xyz[2], vel[0], 0, 0], dtype=np.float32)

        # 协方差矩阵 P (6x6)
        # 初始不确定性较大，给予对角初始化
        self.P = np.diag([2.0, 2.0, 2.0, 2.0, 2.0, 2.0]).astype(np.float32)

        self.lost_cnt = 0
        self.max_lost = 8
        self.skel_smooth = None  # 用于骨架平滑

    def predict(self):
        """
        预测步骤:
        x = F @ x
        P = F @ P @ F.T + Q
        """
        # 1. 状态预测 (通用矩阵乘法在numpy中已经高度优化)
        self.x = self._F @ self.x

        # 2. 协方差预测
        self.P = self._F @ self.P @ self._F.T + self._Q

    def update(self, z):
        """
        更新步骤 (利用矩阵稀疏性优化):
        y = z - H @ x  =>  y = z - x[:3]
        S = H @ P @ H.T + R => S = P[:3, :3] + R
        K = P @ H.T @ inv(S)
        x = x + K @ y
        P = (I - K @ H) @ P => P = P - K @ (H @ P) => P = P - K @ P[:3, :]
        """
        # 1. 计算残差 y (利用 H 的特性直接切片)
        y = z - self.x[:3]

        # 2. 计算残差协方差 S (直接取 P 的左上角 3x3)
        S = self.P[:3, :3] + self._R

        # 3. 计算卡尔曼增益 K (P @ H.T 等价于取 P 的前 3 列)
        # K (6x3) = P[:, :3] @ inv(S)
        # 使用 solve 往往比 inv 更稳定，但对于 3x3 矩阵，inv 差别不大
        S_inv = np.linalg.inv(S)
        K = self.P[:, :3] @ S_inv

        # 4. 更新状态
        self.x = self.x + K @ y

        # 5. 更新协方差 (优化: P = P - K @ (H @ P))
        # H @ P 就是 P 的前 3 行
        self.P = self.P - K @ self.P[:3, :]

        self.lost_cnt = 0

    def get_pos(self):
        return self.x[:3]

    def get_vel(self):
        return self.x[3:]
