#pragma once
#include "Common.h"

typedef int ClusterSerialNumber;
typedef int PointSerialNumber;
typedef float FeatureValue;

class CSizeOfElevationDifferenceCluster
{
public:
	CSizeOfElevationDifferenceCluster(const PointCloudT::Ptr& vCloud,const std::unordered_map<PointSerialNumber, float>& vElevationDifferences) :m_pCloud(vCloud),
		m_ElevationDifferences(vElevationDifferences) {};
	void compute(std::unordered_map<PointSerialNumber, FeatureValue>& voSizeOfElevationDifferenceCluster);

private:
	_NODISCARD bool __initializeKdTree();
	_NODISCARD bool __searchNeighbours(const PointT& vPoint, std::vector<int>& voNeighbourPointIdxs) const;
	_NODISCARD bool __checkNeighbours(const std::vector<int>& vNeighbourPointIndexs) const;

	void __clusterByElevationDifference(std::unordered_multimap<ClusterSerialNumber,PointSerialNumber>& voClusters,
		std::unordered_map<PointSerialNumber, ClusterSerialNumber>& voPoints);

	void __computeClusterFeatureValues(const std::unordered_multimap<ClusterSerialNumber, PointSerialNumber>& vClusters,
		std::unordered_map<ClusterSerialNumber, FeatureValue>& voClusterMapFeatureValue);

private:
	int m_NeighbourNum = 6;
	float m_DifferenceThreshold = 0.3f;
	float m_GroundMaxThreshold = 0.5f;
	PointCloudT::Ptr m_pCloud;
	pcl::search::KdTree<PointT>::Ptr m_pKdTree;
	std::unordered_map<PointSerialNumber, float> m_ElevationDifferences;
	
};