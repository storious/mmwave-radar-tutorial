#define UTEST_IMPLEMENTATION
#include "gtrack.h"
#include "utest.h"

// ==============================
// 测试 1：配置与初始化
// ==============================
TEST(test_gtrack_config_init) {
  Gtrack_Config cfg = {.minSNR = 10.0f,
                       .staticSpeedThr = 0.1f,
                       .clusterRangeThr = 0.5f,
                       .minPointsPerCluster = 3,
                       .distGate = 1.0f,
                       .velGate = 0.5f,
                       .initFrames = 2,
                       .loseFrames = 5};

  EXPECT_EQ(cfg.minSNR, 10.0f);
  EXPECT_EQ(cfg.minPointsPerCluster, 3);
  return EXIT_SUCCESS;
}

// ==============================
// 测试 2：点云预处理
// ==============================
TEST(test_gtrack_preprocess) {
  Gtrack_Config cfg = {.minSNR = 10.0f, .staticSpeedThr = 0.1f};

  Gtrack_Point inputPoints[3] = {
      {1, 2, 3, 0.2f, 15.0f}, // 有效
      {4, 5, 6, 0.05f, 5.0f}, // 静态点，过滤
      {7, 8, 9, 0.3f, 20.0f}  // 有效
  };

  Gtrack_Point outputPoints[3];
  int outputNum = 0;

  Gtrack_preprocess(&cfg, inputPoints, 3, outputPoints, &outputNum);

  EXPECT_EQ(outputNum, 2);
  return EXIT_SUCCESS;
}

// ==============================
// 测试 3：聚类功能
// ==============================
TEST(test_gtrack_cluster) {
  Gtrack_Config cfg = {
      .clusterRangeThr = 0.5f,
      .minPointsPerCluster = 2 // 至少2个点才算有效簇
  };

  Gtrack_Point points[4] = {
      {0, 0, 0, 1.0f, 20},   // 簇A
      {0.1, 0, 0, 1.0f, 20}, // 簇A
      {2, 0, 0, 1.0f, 20},   // 簇B
      {2.1, 0, 0, 1.0f, 20}, // 簇B
  };

  Gtrack_Cluster clusters[10];
  int clusterNum = 0;

  Gtrack_cluster(&cfg, points, 4, clusters, &clusterNum);

  // 正确断言：应该得到 2 个簇
  EXPECT_EQ(clusterNum, 2);
  return EXIT_SUCCESS;
}

// ==============================
// 测试 4：轨迹预测
// ==============================
TEST(test_gtrack_predict) {
  Gtrack_Config cfg = {0};
  Gtrack_Track track = {0};
  track.pos[0] = 1.0f;
  track.vel[0] = 0.5f;

  Gtrack_predict(&cfg, &track, 1, 0.1f);

  EXPECT_NEAR(track.pos[0], 1.05f, 0.02f);
  return EXIT_SUCCESS;
}

// ==============================
// 测试 5：完整帧处理
// ==============================
TEST(test_gtrack_process_frame) {
  Gtrack_Config cfg = {.minSNR = 5,
                       .staticSpeedThr = 0.1f,
                       .clusterRangeThr = 0.5f,
                       .minPointsPerCluster = 2,
                       .initFrames = 2,
                       .loseFrames = 3};

  Gtrack_Point points[4] = {{2.0f, 0, 0, 0.5f, 20},
                            {2.1f, 0, 0, 0.5f, 20},
                            {2.2f, 0, 0, 0.5f, 20},
                            {5.0f, 5.0f, 0, 1.0f, 20}};

  Gtrack_Track tracks[10];
  int trackNum = 0;

  Gtrack_processFrame(&cfg, points, 4, 0.1f, tracks, &trackNum);

  EXPECT_TRUE(trackNum < 2);
  return EXIT_SUCCESS;
}

// ==============================
// 主函数
// ==============================
int main(void) {
  // char *argv[] = {"test_gtrack_config_init", "test_gtrack_preprocess"};
  // RUN_TESTS(2, argv);
  RUN_ALL;
  return 0;
}
