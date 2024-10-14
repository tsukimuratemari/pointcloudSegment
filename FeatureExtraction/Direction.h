#pragma once
#include "Common.h"
#include <pcl/segmentation/supervoxel_clustering.h>

class CSupervoxelDirection {
public:
	CSupervoxelDirection() = default;
	~CSupervoxelDirection() = default;

	void compute(const std::map <std::uint32_t, pcl::Supervoxel<PointT>::Ptr >& vSupervoxels, std::map<std::uint32_t, double>& voSVDirections);
};