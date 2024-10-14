#pragma once
#include "Common.h"

class CColorEntropy
{
public:
	CColorEntropy(const PointCloudT::Ptr& vCloud);

	~CColorEntropy() = default;

	void compute(std::unordered_map<int, float>& voColorEntroys, float viNeighbourhoodRadius = 0.0f);

private:
	PointCloudT::Ptr m_pCloud = nullptr;

	pcl::KdTreeFLANN<PointT>::Ptr m_pSearchTree = nullptr;
	
	float __computeColorVarience(std::vector<int>& viPointIndex);

	void __normalizeColorEntropy(std::unordered_map<int, float>& vioColorEntropy);
};