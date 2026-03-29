#include "gtrack.h"

void Gtrack_preprocess(const Gtrack_Config *cfg,
                       const Gtrack_Point *inputPoints, int inputNum,
                       Gtrack_Point *outputPoints, int *outputNum) {
  *outputNum = 0;

  for (int i = 0; i < inputNum; i++) {
    const Gtrack_Point *p = &inputPoints[i];

    // 1. 过滤低信噪比点
    if (p->snr < cfg->minSNR)
      continue;

    // 2. 过滤静态点（速度接近0）
    if (fabsf(p->velocity) < cfg->staticSpeedThr)
      continue;

    // 保留有效动态点
    outputPoints[(*outputNum)++] = *p;
  }
}
