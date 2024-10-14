#pragma once
#include "Common.h"
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/segmentation/lccp_segmentation.h>

class LCCP
{
public:
	LCCP() = default;
	~LCCP() = default;

	void process(std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr>& vSupervoxelClusters, 
		std::multimap<std::uint32_t, std::uint32_t>& vSupervoxelAdjacency,
		float vVoxelResolution,float vSeedResolution);
	void getConcaveAndConvexAdjacencyList(pcl::LCCPSegmentation<PointT>::SupervoxelAdjacencyList& vSuperVoxelConcaveAndConvexAdjacencyList);



private:
	pcl::LCCPSegmentation<PointT> LCCPOperator;
	bool IsSegment = false;
};