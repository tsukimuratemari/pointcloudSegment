//
// pch.h
//

#pragma once

#include "gtest/gtest.h"

#include <omp.h>
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/features/fpfh.h>
#include <winsock.h>
#include <filesystem>
#include <iomanip>
#include <functional>

#include "common/Product.h"
#include "common/Singleton.h"
#include "common/CommonMicro.h"
#include "common/CommonInterface.h"
#include "common/EventLoggerInterface.h"
#include "common/DesignPatternInterface.h"
#include "common/HiveConfig.h"
#include "common/ConfigInterface.h"
#include "common/UtilityInterface.h"

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointNCloudT;
typedef pcl::PointXYZL PointLT;
typedef pcl::PointCloud<PointLT> PointLCloudT;
typedef pcl::PointXYZ PointCoord;
typedef pcl::PointCloud<PointCoord> PointCloudCoord;