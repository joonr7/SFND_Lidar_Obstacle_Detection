/* \author Aaron Brown */
/* \editor Joonyoung */
#ifndef CLUSTER3D_H_
#define CLUSTER3D_H_

#include <chrono>
#include <string>
#include "kdtree3d.h"

void clusterHelper(int indice, std::vector<std::vector<float>> points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol);
std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol);

#endif
