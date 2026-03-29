#include "gtrack.h"
#include <string.h>

static Gtrack_Track g_trackList[GTRACK_MAX_TRACKS];
static int g_trackNum = 0;

void Gtrack_processFrame(Gtrack_Config *cfg, const Gtrack_Point *inputPoints,
                         int pointNum, float deltaT, Gtrack_Track *outputTracks,
                         int *outputTrackNum) {
  // 临时缓冲区
  Gtrack_Point filtered[GTRACK_MAX_POINTS];
  int filteredNum = 0;

  Gtrack_Cluster clusters[GTRACK_MAX_CLUSTERS];
  int clusterNum = 0;

  Gtrack_Association assoc[GTRACK_MAX_TRACKS];
  int assocNum = 0;

  int unassocTrack[GTRACK_MAX_TRACKS], unassocTrackNum = 0;
  int unassocCls[GTRACK_MAX_CLUSTERS], unassocClsNum = 0;

  // ================= 执行全流程 =================
  Gtrack_preprocess(cfg, inputPoints, pointNum, filtered, &filteredNum);
  Gtrack_cluster(cfg, filtered, filteredNum, clusters, &clusterNum);
  Gtrack_predict(cfg, g_trackList, g_trackNum, deltaT);

  Gtrack_associate(cfg, g_trackList, g_trackNum, clusters, clusterNum, assoc,
                   &assocNum, unassocTrack, &unassocTrackNum, unassocCls,
                   &unassocClsNum);

  Gtrack_update(cfg, assoc, assocNum, unassocTrack, unassocTrackNum, unassocCls,
                unassocClsNum, clusters, clusterNum, g_trackList, &g_trackNum);

  // 输出结果
  memcpy(outputTracks, g_trackList, sizeof(Gtrack_Track) * g_trackNum);
  *outputTrackNum = g_trackNum;
}
