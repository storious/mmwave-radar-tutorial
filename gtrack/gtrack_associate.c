#include "gtrack.h"

void Gtrack_associate(const Gtrack_Config *cfg, const Gtrack_Track *predTracks,
                      int trackNum, const Gtrack_Cluster *clusters,
                      int clusterNum, Gtrack_Association *assocList,
                      int *assocNum, int *unassociatedTrackIdx,
                      int *unassociatedTrackNum, int *unassociatedClusterIdx,
                      int *unassociatedClusterNum) {
  *assocNum = 0;
  *unassociatedTrackNum = 0;
  *unassociatedClusterNum = 0;

  uint8_t trackUsed[GTRACK_MAX_TRACKS] = {0};
  uint8_t clusterUsed[GTRACK_MAX_CLUSTERS] = {0};

  // 遍历所有轨迹-簇对
  for (int t = 0; t < trackNum; t++) {
    const Gtrack_Track *trk = &predTracks[t];
    float bestScore = 1e9;
    int bestCluster = -1;

    for (int c = 0; c < clusterNum; c++) {
      const Gtrack_Cluster *cls = &clusters[c];

      float dx = trk->pos[0] - cls->centerX;
      float dy = trk->pos[1] - cls->centerY;
      float dist = sqrtf(dx * dx + dy * dy);
      float velDiff = fabsf(trk->vel[0] - cls->avgVelocity);

      if (dist > cfg->distGate || velDiff > cfg->velGate)
        continue;

      float score = dist + velDiff * 2;
      if (score < bestScore) {
        bestScore = score;
        bestCluster = c;
      }
    }

    if (bestCluster >= 0) {
      assocList[(*assocNum)++] =
          (Gtrack_Association){t, bestCluster, bestScore};
      trackUsed[t] = 1;
      clusterUsed[bestCluster] = 1;
    }
  }

  // 未关联轨迹
  for (int t = 0; t < trackNum; t++) {
    if (!trackUsed[t])
      unassociatedTrackIdx[(*unassociatedTrackNum)++] = t;
  }

  // 未关联簇（新生目标）
  for (int c = 0; c < clusterNum; c++) {
    if (!clusterUsed[c])
      unassociatedClusterIdx[(*unassociatedClusterNum)++] = c;
  }
}
