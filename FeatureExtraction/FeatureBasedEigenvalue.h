#pragma once
#include "Common.h"
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/segmentation/lccp_segmentation.h>

class CFeatureBasedEigenvalue {
public:
	CFeatureBasedEigenvalue() = default;
	~CFeatureBasedEigenvalue() = default;

	void compute(const pcl::LCCPSegmentation<PointT>::SupervoxelAdjacencyList& vSupervoxelConvexityAdjacencyList,
		const std::map <std::uint32_t, pcl::Supervoxel<PointT>::Ptr >& vSupervoxels,
		std::map<std::uint32_t, SFeaturesBasedEigenvalue>& voFeaturesBasedEigenvalue);

private:
	void __comCloudEigenvalue(const PointCloudT::Ptr& vCloud, std::array<double, 3>& voEigenvalues);
	void __genSupervoxelNeighborhoodCloud(const pcl::LCCPSegmentation<PointT>::SupervoxelAdjacencyList& vSupervoxelConvexityAdjacencyList,
		const std::map <std::uint32_t, pcl::Supervoxel<PointT>::Ptr >& vSupervoxels,
		const std::set<void*>::iterator vVertexItr,
		PointCloudT& voNeighborhoodCloud);
};