import numpy as np


class SkeletonGenerator:
    def __init__(self, std_height=1.75):
        self.std_height = std_height

    def generate(self, cluster_points, tracker_obj):
        # ============================
        # 1. 核心基准：无条件使用 EKF 状态
        # ============================
        # 只要跟踪器存在，骨架就存在。这是解决“骨架消失”的关键。
        pos = tracker_obj.get_pos()
        base_x = pos[0]
        base_y = pos[1]

        # EKF 的 Z 是簇中心的 Z (通常在身体中部，约 0m 左右)
        # 我们用它作为基准参考
        ekf_z = pos[2]

        # ============================
        # 2. 高度 Z 的智能推算
        # ============================
        # 默认假设：脚在地面 (相对于雷达 z = -0.5)
        # 或者根据 EKF 高度推算：脚 = EKF中心 - 身高/2
        default_base_z = -0.5

        # 如果有点云，尝试修正脚的位置
        if len(cluster_points) > 5:
            z_vals = cluster_points[:, 2]
            z_min = np.min(z_vals)

            # 如果点云最低点比默认地面高，说明人可能悬空或者点云不全
            # 此时使用点云最低点作为脚底，更贴合实际点云
            # 但要防止点云噪点太高导致人“飞起来”
            # 策略：取 (点云最低点) 和 (EKF中心 - 0.8m) 中的较低者
            base_z = min(z_min, ekf_z - 0.8)
        else:
            # 没有点云时，完全信任 EKF 和 假设
            base_z = default_base_z

        # ============================
        # 3. 生成刚性骨架模板
        # ============================
        # 使用标准人体比例，不再“强行拟合”点云的每一个细节
        H = self.std_height  # 1.75m

        # 定义关节点相对于脚底 (base_z) 的高度比例
        # 这样骨架就是一个刚性的整体，不会变形
        skel_new = {
            "head": np.array([base_x, base_y, base_z + H * 1.0]),
            "neck": np.array([base_x, base_y, base_z + H * 0.85]),
            "shoulderL": np.array([base_x - 0.2, base_y, base_z + H * 0.80]),
            "shoulderR": np.array([base_x + 0.2, base_y, base_z + H * 0.80]),
            "elbowL": np.array([base_x - 0.35, base_y, base_z + H * 0.60]),
            "elbowR": np.array([base_x + 0.35, base_y, base_z + H * 0.60]),
            "wristL": np.array([base_x - 0.4, base_y, base_z + H * 0.40]),
            "wristR": np.array([base_x + 0.4, base_y, base_z + H * 0.40]),
            "hipL": np.array([base_x - 0.15, base_y, base_z + H * 0.50]),
            "hipR": np.array([base_x + 0.15, base_y, base_z + H * 0.50]),
            "kneeL": np.array([base_x - 0.15, base_y, base_z + H * 0.25]),
            "kneeR": np.array([base_x + 0.15, base_y, base_z + H * 0.25]),
            "ankleL": np.array([base_x - 0.15, base_y, base_z + H * 0.05]),
            "ankleR": np.array([base_x + 0.15, base_y, base_z + H * 0.05]),
        }

        # ============================
        # 4. 平滑处理 (XY 跟随 EKF，无需平滑)
        # ============================
        if tracker_obj.skel_smooth is None:
            tracker_obj.skel_smooth = skel_new
        else:
            # XY 轴：完全跟随 EKF (alpha = 1.0)
            # Z 轴：稍微平滑 (alpha = 0.4)，防止高度突变
            alpha_xy = 1.0
            alpha_z = 0.4

            for k in skel_new.keys():
                old_pt = tracker_obj.skel_smooth[k]
                new_pt = skel_new[k]

                # 直接线性插值
                smooth_x = old_pt[0] * (1 - alpha_xy) + new_pt[0] * alpha_xy
                smooth_y = old_pt[1] * (1 - alpha_xy) + new_pt[1] * alpha_xy
                smooth_z = old_pt[2] * (1 - alpha_z) + new_pt[2] * alpha_z

                tracker_obj.skel_smooth[k] = np.array([smooth_x, smooth_y, smooth_z])

        return tracker_obj.skel_smooth
