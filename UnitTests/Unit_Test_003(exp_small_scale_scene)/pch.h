//
// pch.h
//

#pragma once

#include "gtest/gtest.h"
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/pfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <iostream>
#include <string>
#include <map>
#include <unordered_map>
#include <random>
#include <optional>
#include <iostream> //标准c++库输入输出相关头文件
#include <vector>
#include <pcl/point_types.h> //pcl中支持的点类型头文件
#include <pcl/io/pcd_io.h> //pcd读写相关头文件
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h> //包含kdtree头文件
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>