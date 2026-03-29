#ifndef GTRACK_H
#define GTRACK_H

#include <math.h>
#include <stdint.h>

// 轨迹状态定义
#define GTRACK_STATE_INIT 0     // 初始化
#define GTRACK_STATE_ACTIVE 1   // 激活
#define GTRACK_STATE_OCCLUDED 2 // 遮挡
#define GTRACK_STATE_DELETE 3   // 删除

// 最大点云/聚类/轨迹数量限制
#define GTRACK_MAX_POINTS 1000
#define GTRACK_MAX_CLUSTERS 50
#define GTRACK_MAX_TRACKS 50

// 配置结构体
typedef struct {
  float minSNR;            // 最小信噪比
  float staticSpeedThr;    // 静态点速度阈值
  float clusterRangeThr;   // 聚类距离阈值
  int minPointsPerCluster; // 簇最小点数
  float distGate;          // 关联距离门限
  float velGate;           // 关联速度门限
  int initFrames;          // 轨迹激活帧数
  int loseFrames;          // 轨迹删除帧数
} Gtrack_Config;

// 点云结构体
typedef struct {
  float x, y, z;
  float velocity;
  float snr;
} Gtrack_Point;

// 聚类结果
typedef struct {
  float centerX, centerY, centerZ;
  float avgVelocity;
  float width, height, depth;
  int pointNum;
} Gtrack_Cluster;

// 轨迹
typedef struct {
  int trackId;
  float pos[3];
  float vel[3];
  float acc[3];
  float size[3];
  int state;
  int consecutiveDetect;
  int consecutiveLose;
} Gtrack_Track;

// 关联结果
typedef struct {
  int trackIdx;
  int clusterIdx;
  float score;
} Gtrack_Association;

// ========================= 模块接口 =========================
// 1. 点云预处理
void Gtrack_preprocess(const Gtrack_Config *cfg,
                       const Gtrack_Point *inputPoints, int inputNum,
                       Gtrack_Point *outputPoints, int *outputNum);

// 2. 聚类
void Gtrack_cluster(const Gtrack_Config *cfg,
                    const Gtrack_Point *filteredPoints, int pointNum,
                    Gtrack_Cluster *outputClusters, int *clusterNum);

// 3. 轨迹预测
void Gtrack_predict(const Gtrack_Config *cfg, Gtrack_Track *trackList,
                    int trackNum, float deltaT);

// 4. 数据关联
void Gtrack_associate(const Gtrack_Config *cfg, const Gtrack_Track *predTracks,
                      int trackNum, const Gtrack_Cluster *clusters,
                      int clusterNum, Gtrack_Association *assocList,
                      int *assocNum, int *unassociatedTrackIdx,
                      int *unassociatedTrackNum, int *unassociatedClusterIdx,
                      int *unassociatedClusterNum);

// 5. 轨迹更新 & 生命周期
void Gtrack_update(Gtrack_Config *cfg, const Gtrack_Association *assocList,
                   int assocNum, const int *unassociatedTrackIdx,
                   int unassocTrackNum, const int *unassociatedClusterIdx,
                   int unassocClusterNum, const Gtrack_Cluster *clusters,
                   int clusterNum, Gtrack_Track *trackList, int *trackNum);

// 6. 顶层帧处理
void Gtrack_processFrame(Gtrack_Config *cfg, const Gtrack_Point *inputPoints,
                         int pointNum, float deltaT, Gtrack_Track *outputTracks,
                         int *outputTrackNum);

#endif
