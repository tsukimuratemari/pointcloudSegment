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
#include <iostream> //��׼c++������������ͷ�ļ�
#include <vector>
#include <pcl/point_types.h> //pcl��֧�ֵĵ�����ͷ�ļ�
#include <pcl/io/pcd_io.h> //pcd��д���ͷ�ļ�
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h> //����kdtreeͷ�ļ�
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>