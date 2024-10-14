#include "pch.h"
#include "DoN.h"

CDoN::CDoN(DoNInputPointCloudType::Ptr& viPointCloud)
{
	m_pOriginPointCloud = std::make_shared<DoNInputPointCloudType>(*viPointCloud) ;
	__movePointCloud2Origin();
	m_pNormalEstimateInputPointCloud = std::make_shared<NormalEstimateInputPointCloudType>();
	pcl::copyPointCloud(*m_pOriginPointCloud, *m_pNormalEstimateInputPointCloud);
	m_pSearchTree = std::make_shared<pcl::search::KdTree<NormalEstimateInputPointType>>();
	m_pSearchTree->setInputCloud(m_pNormalEstimateInputPointCloud);
}

void CDoN::__movePointCloud2Origin()
{
	DoNInputPointType MinPoint, MaxPoint;
	pcl::getMinMax3D(*m_pOriginPointCloud, MinPoint, MaxPoint);
	for (int i = 0; i < m_pOriginPointCloud->size(); i++)
	{
		m_pOriginPointCloud->points[i].x -= MinPoint.x;
		m_pOriginPointCloud->points[i].y -= MinPoint.y;
		m_pOriginPointCloud->points[i].z -= MinPoint.z;
	}
}

void CDoN::__estimatePointCloudNormal(NormalEstimateOutputPointCloudType::Ptr voNormal, const float viNormalScale)
{
	pcl::NormalEstimationOMP<NormalEstimateInputPointType, NormalType> NormalEstimation;
	NormalEstimation.setInputCloud(m_pNormalEstimateInputPointCloud);
	NormalEstimation.setSearchMethod(m_pSearchTree);
	double PointCloudCoordCenterX = 0.0f, PointCloudCoordCenterY = 0.0f;
	NormalEstimation.setViewPoint(PointCloudCoordCenterX, PointCloudCoordCenterY, std::numeric_limits<float>::max());
	NormalEstimation.setRadiusSearch(viNormalScale);
	NormalEstimation.compute(*voNormal);
}

void CDoN::computeDoN(const float vSmallScale, const float vLargeScale, std::unordered_map<int, float>& voDONFeatureMap)
{
	NormalEstimateOutputPointCloudType::Ptr SmallScaleNormal = std::make_shared<NormalEstimateOutputPointCloudType>();
	__estimatePointCloudNormal(SmallScaleNormal, vSmallScale);
	NormalEstimateOutputPointCloudType::Ptr LargeScaleNormal = std::make_shared<NormalEstimateOutputPointCloudType>();
	__estimatePointCloudNormal(LargeScaleNormal, vLargeScale);

	pcl::DifferenceOfNormalsEstimation<DoNInputPointType, NormalType, NormalType> DON;
	DON.setInputCloud(m_pOriginPointCloud);
	DON.setNormalScaleLarge(LargeScaleNormal);
	DON.setNormalScaleSmall(SmallScaleNormal);
	if (!DON.initCompute())
	{
		std::cerr << "Error: Could not initialize DoN feature operator" << std::endl;
		exit(EXIT_FAILURE);
	}
	NormalEstimateOutputPointCloudType::Ptr DONFeature = std::make_shared<NormalEstimateOutputPointCloudType>();
	pcl::copyPointCloud(*LargeScaleNormal, *DONFeature);
	DON.computeFeature(*DONFeature);

	voDONFeatureMap.clear();
	for (int i = 0; i < DONFeature->points.size(); i++)
	{
		if (DONFeature->points.at(i).curvature > 0.92f)DONFeature->points.at(i).curvature = 1.0f - DONFeature->points.at(i).curvature;
		voDONFeatureMap.insert(std::make_pair(i, DONFeature->points.at(i).curvature));

	}
		
}