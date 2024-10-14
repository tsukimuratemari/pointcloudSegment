#pragma once
#include "Common.h"
#include <pcl/segmentation/supervoxel_clustering.h>

class CSemanticCategory {
public:
	CSemanticCategory() = default;
	~CSemanticCategory() = default;

	void compute(const PointCloudT::Ptr& vOriginCloud, const std::map<std::uint32_t, pcl::Supervoxel<PointT>::Ptr>& vioSupervoxels);
private:
	void __checkLabel(std::map<std::uint32_t, std::uint32_t>& vLabelCount, const std::uint32_t vLabel);
	std::uint32_t __findMaxCountLabel(const std::map<std::uint32_t, std::uint32_t>& vLabelCount);
};