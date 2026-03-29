#include "gtrack.h"
#include <string.h>

static int generateTrackId(void) {
  static int id = 1;
  return id++;
}

void Gtrack_update(Gtrack_Config *cfg, const Gtrack_Association *assocList,
                   int assocNum, const int *unassociatedTrackIdx,
                   int unassocTrackNum, const int *unassociatedClusterIdx,
                   int unassocClusterNum, const Gtrack_Cluster *clusters,
                   int clusterNum, Gtrack_Track *trackList, int *trackNum) {
  // 1. 更新已关联轨迹
  for (int i = 0; i < assocNum; i++) {
    int tIdx = assocList[i].trackIdx;
    int cIdx = assocList[i].clusterIdx;
    const Gtrack_Cluster *c = &clusters[cIdx];
    Gtrack_Track *t = &trackList[tIdx];

    t->pos[0] = c->centerX;
    t->pos[1] = c->centerY;
    t->pos[2] = c->centerZ;
    t->vel[0] = c->avgVelocity;
    t->consecutiveDetect++;
    t->consecutiveLose = 0;

    if (t->consecutiveDetect >= cfg->initFrames)
      t->state = GTRACK_STATE_ACTIVE;
  }

  // 2. 未关联轨迹（遮挡/删除）
  for (int i = 0; i < unassocTrackNum; i++) {
    int tIdx = unassociatedTrackIdx[i];
    Gtrack_Track *t = &trackList[tIdx];
    t->consecutiveLose++;
    t->consecutiveDetect = 0;

    if (t->consecutiveLose >= cfg->loseFrames)
      t->state = GTRACK_STATE_DELETE;
    else
      t->state = GTRACK_STATE_OCCLUDED;
  }

  // 3. 新生轨迹
  for (int i = 0; i < unassocClusterNum; i++) {
    int cIdx = unassociatedClusterIdx[i];
    const Gtrack_Cluster *c = &clusters[cIdx];

    if (*trackNum >= GTRACK_MAX_TRACKS)
      break;

    Gtrack_Track *t = &trackList[*trackNum];
    memset(t, 0, sizeof(Gtrack_Track));
    t->trackId = generateTrackId();
    t->pos[0] = c->centerX;
    t->pos[1] = c->centerY;
    t->pos[2] = c->centerZ;
    t->vel[0] = c->avgVelocity;
    t->state = GTRACK_STATE_INIT;
    t->consecutiveDetect = 1;
    (*trackNum)++;
  }

  // 4. 清理删除状态的轨迹
  int validTracks = 0;
  for (int i = 0; i < *trackNum; i++) {
    if (trackList[i].state != GTRACK_STATE_DELETE)
      trackList[validTracks++] = trackList[i];
  }
  *trackNum = validTracks;
}
