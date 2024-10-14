#include "pch.h"
#include "Covariance.h"

CCovariance::CCovariance(PointCloudT::Ptr viPointCloud)
{
	m_pOriginPointCloud = std::make_shared<PointCloudCoord>();
	pcl::copyPointCloud(*viPointCloud, *m_pOriginPointCloud);
};

void CCovariance::computeCovariance(const float vScale, std::unordered_map<int, std::vector<float>>& voCovFeatureMap)
{
	pcl::KdTreeFLANN<PointCoord> KDTree;
	KDTree.setInputCloud(m_pOriginPointCloud);

	std::vector<int> NeighboutIndex;
	std::vector<float> NeighbourDistance;

	std::unordered_map<int, float> RelativeProjectHeight;
	float ProjectScale = 0.5f;
	__computeRelativeHeight(ProjectScale, RelativeProjectHeight);

	std::unordered_map<int, float> AbsoluteProjectHeight;
	__computeAbsoluteHeight(AbsoluteProjectHeight);

	for (int i = 0; i < m_pOriginPointCloud->points.size(); i++)
	{
		if (KDTree.radiusSearch(m_pOriginPointCloud->points[i], vScale, NeighboutIndex, NeighbourDistance) > 0)
		{
			Eigen::Matrix<double, 4, 1>CentroId;
			pcl::compute3DCentroid(*m_pOriginPointCloud, NeighboutIndex, CentroId);
			Eigen::Matrix<double, 3, 3>CovarianceMatrix;
			pcl::computeCovarianceMatrix(*m_pOriginPointCloud, NeighboutIndex, CentroId, CovarianceMatrix);

			Eigen::Matrix3d EigenVectors;
			Eigen::Vector3d EigenValues;
			pcl::eigen33(CovarianceMatrix, EigenVectors, EigenValues);
			std::vector<float> SinglePointCovFeature;
			__computeSinglePointCovariance(EigenValues, SinglePointCovFeature);
			//最后加上相对高程结合平面度的楼顶特征
			SinglePointCovFeature.push_back(SinglePointCovFeature[1] *= RelativeProjectHeight.at(i));
			//SinglePointCovFeature.push_back(SinglePointCovFeature[1] * RelativeProjectHeight.at(i));
			//SinglePointCovFeature.push_back(SinglePointCovFeature[1] * AbsoluteProjectHeight.at(i));
			voCovFeatureMap.insert(std::make_pair(i, SinglePointCovFeature));
		}
	}
};

void CCovariance::__computeAbsoluteHeight(std::unordered_map<int, float>& voRelativeHeightMap)
{
	float Max = 1e-8, Min = FLT_MAX;
	for (int i = 0; i < m_pOriginPointCloud->points.size(); i++)
	{
		if (Max < m_pOriginPointCloud->points[i].z)Max = m_pOriginPointCloud->points[i].z;
		if (Min > m_pOriginPointCloud->points[i].z)Min = m_pOriginPointCloud->points[i].z;
	}
	float HeightDelta = Max - Min;
	for (int i = 0; i < m_pOriginPointCloud->points.size(); i++)
	{
		voRelativeHeightMap.insert(std::make_pair(i, (m_pOriginPointCloud->points[i].z - Min) / HeightDelta));
	}
}

void CCovariance::__computeRelativeHeight(const float vScale, std::unordered_map<int, float>& voRelativeHeightMap)
{
	PointCloudCoord::Ptr ProjectedCloud(new PointCloudCoord);
	pcl::ModelCoefficients::Ptr Coefficients(new pcl::ModelCoefficients());
	Coefficients->values.push_back(0.0);
	Coefficients->values.push_back(0.0);
	Coefficients->values.push_back(1.0);
	Coefficients->values.push_back(0.0);
	pcl::ProjectInliers<PointCoord> ProjectInlier;
	ProjectInlier.setInputCloud(m_pOriginPointCloud);
	ProjectInlier.setModelCoefficients(Coefficients);
	ProjectInlier.setModelType(pcl::SACMODEL_PLANE);
	ProjectInlier.filter(*ProjectedCloud);
	pcl::KdTreeFLANN<PointCoord> KDTree;
	KDTree.setInputCloud(ProjectedCloud);
	std::vector<int> NeighboutIndex;
	std::vector<float> NeighbourDistance;

	float Max = 1e-8, Min = FLT_MAX;
	for (int i = 0; i < m_pOriginPointCloud->points.size(); i++)
	{
		if (Max < m_pOriginPointCloud->points[i].z)Max = m_pOriginPointCloud->points[i].z;
		if (Min > m_pOriginPointCloud->points[i].z)Min = m_pOriginPointCloud->points[i].z;
	}
	float HeightDelta = Max - Min;
	for (int i = 0; i < ProjectedCloud->points.size(); i++)
	{
		if (KDTree.radiusSearch(ProjectedCloud->points[i], vScale, NeighboutIndex, NeighbourDistance) > 0)
		{
			float ProjectedHeight = 0.0f;
			for (int i = 0; i < NeighboutIndex.size(); i++)
			{
				ProjectedHeight += m_pOriginPointCloud->points[NeighboutIndex[i]].z;
			}
			ProjectedHeight /= NeighboutIndex.size();
			voRelativeHeightMap.insert(std::make_pair(i, (ProjectedHeight - Min) / HeightDelta));
		}
	}
}

void CCovariance::__computeSinglePointCovariance(Eigen::Vector3d& viEigenValues, std::vector<float>& voSinglePointCovFeature)
{
	Eigen::Vector3d::Index MaxRow, MaxCol, MinRow, MinCol;
	viEigenValues.maxCoeff(&MaxRow, &MaxCol);
	viEigenValues.minCoeff(&MinRow, &MinCol);
	
	float Lambda1 = viEigenValues[MaxRow];
	int MidRow = (3 - MaxRow - MinRow) >= 0 ? (3 - MaxRow - MinRow) : 0;
	MidRow= (3 - MaxRow - MinRow) < viEigenValues.size() ? (3 - MaxRow - MinRow) : 0;
	float Lambda2 = viEigenValues[MidRow];
	float Lambda3 = viEigenValues[MinRow];
	if (Lambda1 < 1e-6f) Lambda1 = 1.0f;
	if (Lambda2 < 1e-6f) Lambda2 = 1.0f;
	if (Lambda3 < 1e-6f) Lambda3 = 1.0f;
	
	float Sum = Lambda1 + Lambda2 + Lambda3;
	Lambda1 /= Sum;
	Lambda2 /= Sum;
	Lambda3 /= Sum;

	//线性
	float Linearity = (Lambda1 - Lambda2) / Lambda1;
	//平面性
	float Planarity = (Lambda2 - Lambda3) / Lambda1;
	//散射
	float Scattering = Lambda3 / Lambda1;
	//各向异性
	float Anisotropy = (Lambda1 - Lambda3) / Lambda1;
	//曲率
	float SurfaceVariation = 3 * Lambda3;

	voSinglePointCovFeature.push_back(Linearity);
	voSinglePointCovFeature.push_back(Planarity);
	voSinglePointCovFeature.push_back(Scattering);
	voSinglePointCovFeature.push_back(Anisotropy);
	voSinglePointCovFeature.push_back(SurfaceVariation);
}