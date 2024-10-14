#pragma once
#include "Common.h"

class CZDirectionDensity
{
public:
	CZDirectionDensity(PointCloudT::Ptr viPointCloud);

	~CZDirectionDensity() = default;

	void computeZDirectionDensity(const float vScale, std::unordered_map<int, float>& voZDirectionDestinyMap);

private:

	PointCloudCoord::Ptr m_pOriginPointCloud = nullptr;


};