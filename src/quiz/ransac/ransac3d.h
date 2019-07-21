/* \author Aaron Brown */
/* \editor Joonyoung */
#ifndef RANSAC3D_H_
#define RANSAC3D_H_

#include <iostream>
#include <vector>
#include <string>
#include <unordered_set>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

// template<typename PointT>
std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceTol);

#endif
