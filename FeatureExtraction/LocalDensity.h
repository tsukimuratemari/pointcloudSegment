#pragma once
#include "Common.h"
#include <pcl/segmentation/supervoxel_clustering.h>

//Q:在采样密度均匀的情况下，是否应该使用radius搜索？是否本身这个特征就很难起效果？
class CLocalDensity {
public:
	CLocalDensity() = default;
	~CLocalDensity() = default;

	void compute(const std::map <std::uint32_t, pcl::Supervoxel<PointT>::Ptr >& vSupervoxels, std::map<std::uint32_t, double>& voLocalDensitys);
};