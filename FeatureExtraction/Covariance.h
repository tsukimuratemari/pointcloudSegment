#pragma once
#include "Common.h"

class CCovariance
{
public:
	CCovariance(PointCloudT::Ptr viPointCloud);

	~CCovariance() = default;

	void computeCovariance(const float vScale, std::unordered_map<int, std::vector<float>>& voCovFeatureMap);

	
private:

	PointCloudCoord::Ptr m_pOriginPointCloud = nullptr;

	void __computeSinglePointCovariance(Eigen::Vector3d& viEigenValues, std::vector<float>& voSinglePointCovFeature);

	void __computeAbsoluteHeight(std::unordered_map<int, float>& voRelativeHeightMap);

	void __computeRelativeHeight(const float vScale, std::unordered_map<int, float>& voRelativeHeightMap);
};
