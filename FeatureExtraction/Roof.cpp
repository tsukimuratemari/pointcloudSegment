#include "pch.h"
#include "Roof.h"
CRoof::CRoof(PointCloudT::Ptr& viPointCloud)
{
	m_pOriginPointCloud = std::make_shared<PointCloudCoord>();
	pcl::copyPointCloud(*viPointCloud, *m_pOriginPointCloud);
}

void CRoof::computeRoofFeature(float vRadius, std::map<int, float> voRoofFeatureMap)
{

}

void CRoof::__computePlanness(float vRadius, std::map<int, float> voPlannessFeatureMap)
{

}