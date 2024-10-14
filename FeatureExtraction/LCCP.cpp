#include"pch.h"
#include"LCCP.h"

void LCCP::process(std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr>& vSupervoxelClusters, 
	std::multimap<std::uint32_t, std::uint32_t>& vSupervoxelAdjacency,
	float vVoxelResolution, float vSeedResolution)
{
	float ConcavityToleranceThreshold = 10;
	float SmoothnessThreshold = 0.1;
	std::uint32_t MinSegmentSize = 0.002;
	std::uint32_t KFactor = 0;
	bool IsExtendedConvexity = false;
	bool IsSanityCriterion = false;

	LCCPOperator.setConcavityToleranceThreshold(ConcavityToleranceThreshold);
	LCCPOperator.setSmoothnessCheck(true, vVoxelResolution, vSeedResolution, SmoothnessThreshold);
	LCCPOperator.setInputSupervoxels(vSupervoxelClusters, vSupervoxelAdjacency);
	LCCPOperator.setMinSegmentSize(MinSegmentSize);
	LCCPOperator.segment();

	IsSegment = true;
}

void LCCP::getConcaveAndConvexAdjacencyList(pcl::LCCPSegmentation<PointT>::SupervoxelAdjacencyList& vSuperVoxelConcaveAndConvexAdjacencyList)
{
	if (!IsSegment)return;
	LCCPOperator.getSVAdjacencyList(vSuperVoxelConcaveAndConvexAdjacencyList);
}