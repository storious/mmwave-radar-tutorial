#include "gtrack.h"
#include <string.h>

static float distance3d(const Gtrack_Point *a, const Gtrack_Point *b) {
  float dx = a->x - b->x;
  float dy = a->y - b->y;
  float dz = a->z - b->z;
  return sqrtf(dx * dx + dy * dy + dz * dz);
}

void Gtrack_cluster(const Gtrack_Config *cfg,
                    const Gtrack_Point *filteredPoints, int pointNum,
                    Gtrack_Cluster *outputClusters, int *clusterNum) {
  *clusterNum = 0;
  uint8_t visited[GTRACK_MAX_POINTS] = {0};

  for (int i = 0; i < pointNum; i++) {
    if (visited[i])
      continue;

    // 新建簇
    int curCluster = (*clusterNum)++;
    Gtrack_Cluster *cls = &outputClusters[curCluster];
    memset(cls, 0, sizeof(Gtrack_Cluster));

    // 区域增长聚类
    int queue[GTRACK_MAX_POINTS], qIdx = 0;
    queue[qIdx++] = i;
    visited[i] = 1;

    float sumX = 0, sumY = 0, sumZ = 0, sumVel = 0;
    int count = 0;

    while (qIdx > 0) {
      int idx = queue[--qIdx];
      const Gtrack_Point *p = &filteredPoints[idx];

      sumX += p->x;
      sumY += p->y;
      sumZ += p->z;
      sumVel += p->velocity;
      count++;

      for (int j = i + 1; j < pointNum; j++) {
        if (!visited[j] &&
            distance3d(p, &filteredPoints[j]) < cfg->clusterRangeThr) {
          visited[j] = 1;
          queue[qIdx++] = j;
        }
      }
    }

    // 簇点数不足则丢弃
    if (count < cfg->minPointsPerCluster) {
      (*clusterNum)--;
      continue;
    }

    // 填充簇信息
    cls->centerX = sumX / count;
    cls->centerY = sumY / count;
    cls->centerZ = sumZ / count;
    cls->avgVelocity = sumVel / count;
    cls->pointNum = count;
  }
}
