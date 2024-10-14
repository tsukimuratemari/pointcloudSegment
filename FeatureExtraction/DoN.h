#pragma once
#include "Common.h"

class CDoN
{
public:
	CDoN(DoNInputPointCloudType::Ptr& viPointCloud);

	~CDoN() = default;

	void computeDoN(const float vSmallScale, const float vLargeScale, std::unordered_map<int, float>&voDONFeatureMap);

private:
	DoNInputPointCloudType::Ptr m_pOriginPointCloud = nullptr;

	NormalEstimateInputPointCloudType::Ptr m_pNormalEstimateInputPointCloud = nullptr;

	pcl::search::Search<NormalEstimateInputPointType>::Ptr m_pSearchTree = nullptr;

	void __movePointCloud2Origin();

	void __estimatePointCloudNormal(NormalEstimateOutputPointCloudType::Ptr voNormal, const float viNormalScale);
};
