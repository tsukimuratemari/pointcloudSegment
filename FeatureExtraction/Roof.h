#pragma once
#include "Common.h"
class CRoof
{
public:
	CRoof(PointCloudT::Ptr& viPointCloud);

	~CRoof() = default;
	
	void computeRoofFeature(float vRadius, std::map<int, float> voRoofFeatureMap);

private:
	PointCloudCoord::Ptr m_pOriginPointCloud = nullptr;

	void __computePlanness(float vRadius, std::map<int, float> voPlannessFeatureMap);

	void __computeRelativeHeight(float vRadius, std::map<int, float> voRelativeHeightFeatureMap);

};