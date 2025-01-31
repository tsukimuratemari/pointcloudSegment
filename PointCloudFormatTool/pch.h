#pragma once
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <iostream>
#include <time.h>
#include <math.h>
#include <unordered_map>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/don.h>
#include <pcl/common/io.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <boost/thread/thread.hpp>
#include <pcl/features/fpfh_omp.h> 
#include <pcl/registration/correspondence_estimation.h>
#include <algorithm>
