import numpy as np
from scipy.optimize import linear_sum_assignment

from ekf import EKFTarget


class ZoneManager:
    def __init__(self, max_range=10.0, fov_deg=60.0):
        self.max_range = max_range
        self.fov = np.radians(fov_deg)
        self.edge_dist_th = 8.0

    def is_valid(self, x, y):
        r = np.sqrt(x**2 + y**2)
        theta = np.arctan2(x, y)
        return (r < self.max_range) and (r > 0.3) and (np.abs(theta) < self.fov / 2)

    def is_entry_point(self, x, y):
        r = x**2 + y**2
        return r > (self.edge_dist_th**2)


class MultiTracker:
    def __init__(self, zone_manager):
        self.trackers = []
        self.next_id = 1
        self.max_dist = 2.5  # 增加关联距离阈值（原来 2.0 可能太严格）
        self.zone = zone_manager
        self.is_first_frame = True

        # 新增：轨迹管理参数
        self.max_trackers = 10  # 最多同时跟踪 10 个目标
        self.min_hits_to_confirm = 2  # 需要连续检测 2 次才确认为新目标

    def associate(self, detections):
        # 1. 预测所有跟踪器
        for t in self.trackers:
            t.predict()

        if len(detections) == 0:
            for t in self.trackers:
                t.lost_cnt += 1
            self.trackers = [t for t in self.trackers if t.lost_cnt < t.max_lost]
            return

        # 2. 构建代价矩阵
        n_trk, n_det = len(self.trackers), len(detections)
        cost = np.zeros((n_trk, n_det))

        for i, t in enumerate(self.trackers):
            t_pos = t.get_pos()
            for j, d in enumerate(detections):
                # 代价 = 距离 + 速度惩罚
                dist = np.linalg.norm(t_pos - d[:3])

                # 速度一致性惩罚（可选）
                # vel_diff = np.linalg.norm(t.get_vel() - [d[3], 0, 0])
                # cost[i, j] = dist + 0.5 * vel_diff

                cost[i, j] = dist

        # 3. 匈牙利算法关联
        row, col = linear_sum_assignment(cost)

        matched_trk = set()
        matched_det = set()

        for i, j in zip(row, col):
            if cost[i, j] < self.max_dist:  # 使用更宽松的阈值
                self.trackers[i].update(detections[j][:3])
                matched_trk.add(i)
                matched_det.add(j)

        # 4. 处理未匹配的检测（可能是新目标）
        for j in range(n_det):
            if j not in matched_det:
                d = detections[j]
                x, y, z = d[0], d[1], d[2]

                # 严格的新目标创建条件
                if not self.zone.is_valid(x, y):
                    continue

                # 限制新目标创建：必须从边缘进入（非首帧）
                if not self.is_first_frame:
                    if not self.zone.is_entry_point(x, y):
                        continue

                # 高度约束：新目标必须在合理高度范围内
                if z < -0.5 or z > 2.5:
                    continue

                # 限制最大目标数量
                if len(self.trackers) >= self.max_trackers:
                    # 移除最旧的或 lost_cnt 最大的跟踪器
                    self.trackers.sort(key=lambda t: t.lost_cnt, reverse=True)
                    if self.trackers[0].lost_cnt > 0:
                        self.trackers.pop(0)
                    else:
                        continue  # 所有目标都很健康，不创建新的

                # 创建新跟踪器
                self.trackers.append(EKFTarget(self.next_id, d[:3], [d[3], 0, 0]))
                self.next_id += 1

        # 5. 清理丢失的跟踪器
        self.trackers = [t for t in self.trackers if t.lost_cnt < t.max_lost]

        # 6. 限制 ID 增长：如果 ID 超过 100，重置
        if self.next_id > 100:
            # 重新编号所有现有跟踪器
            for i, t in enumerate(self.trackers):
                t.id = i + 1
            self.next_id = len(self.trackers) + 1

        self.is_first_frame = False
