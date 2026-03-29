#include "gtrack.h"

void Gtrack_predict(const Gtrack_Config *cfg, Gtrack_Track *trackList,
                    int trackNum, float deltaT) {
  for (int i = 0; i < trackNum; i++) {
    Gtrack_Track *t = &trackList[i];

    if (t->state == GTRACK_STATE_DELETE)
      continue;

    // 匀速模型：x = x + v * dt
    t->pos[0] += t->vel[0] * deltaT;
    t->pos[1] += t->vel[1] * deltaT;
    t->pos[2] += t->vel[2] * deltaT;
  }
}
