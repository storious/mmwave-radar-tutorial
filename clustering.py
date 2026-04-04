import numpy as np
from sklearn.cluster import DBSCAN


class AdaptiveClusterer:
    def __init__(self):
        # 距离分段参数: (min_range, max_range, eps, min_samples)
        self.range_bins = [
            (0.0, 4.0, 0.5, 4),  # 近距离：点云密，eps小，防止粘连
            (4.0, 8.0, 0.8, 4),  # 中距离
            (8.0, 15.0, 1.2, 3),  # 远距离：点云稀疏，eps大，防止分裂
        ]

    def filter_points(self, pc):
        """基础点云过滤"""
        if len(pc) == 0:
            return pc
        snr_ok = pc[:, 4] > 10.0
        r = np.linalg.norm(pc[:, :3], axis=1)
        range_ok = (r > 0.5) & (r < 12.0)
        vel_ok = np.abs(pc[:, 3]) < 2.5
        z_ok = (pc[:, 2] > -0.5) & (pc[:, 2] < 1.5)
        return pc[snr_ok & range_ok & vel_ok & z_ok]

    def cluster(self, pc):
        """距离自适应 DBSCAN"""
        if len(pc) < 5:
            return [], []

        det_centers, smpl_blocks = [], []
        ranges = np.linalg.norm(pc[:, :2], axis=1)

        # 分段聚类
        for r_min, r_max, eps, min_pts in self.range_bins:
            mask = (ranges >= r_min) & (ranges < r_max)
            if np.sum(mask) < min_pts:
                continue

            sub_pc = pc[mask]
            cls = DBSCAN(eps=eps, min_samples=min_pts).fit(sub_pc[:, :2])
            labels = cls.labels_

            for lab in np.unique(labels):
                if lab == -1:
                    continue
                cluster_points = sub_pc[labels == lab]

                # 高度过滤
                if np.mean(cluster_points[:, 2]) > 2.0:
                    continue

                # 加权质心
                weights = cluster_points[:, 4]
                center = np.average(cluster_points[:, :3], axis=0, weights=weights)
                vel = np.mean(cluster_points[:, 3])

                det_centers.append([center[0], center[1], center[2], vel])
                smpl_blocks.append(cluster_points)

        return det_centers, smpl_blocks
