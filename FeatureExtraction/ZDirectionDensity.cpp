#include "pch.h"
#include "ZDirectionDensity.h"

CZDirectionDensity::CZDirectionDensity(PointCloudT::Ptr viPointCloud)
{
	m_pOriginPointCloud = std::make_shared<PointCloudCoord>();
	pcl::copyPointCloud(*viPointCloud, *m_pOriginPointCloud);
}

void CZDirectionDensity::computeZDirectionDensity(const float vScale, std::unordered_map<int, float>& voZDirectionDestinyMap)
{

	PointCloudCoord::Ptr ProjectedCloud(new PointCloudCoord);
	pcl::ModelCoefficients::Ptr Coefficients(new pcl::ModelCoefficients());
	Coefficients->values.push_back(0.0);
	Coefficients->values.push_back(0.0);
	Coefficients->values.push_back(1.0);
	Coefficients->values.push_back(0.0);

	pcl::ProjectInliers<pcl::PointXYZ> ProjectInlier;
	ProjectInlier.setInputCloud(m_pOriginPointCloud);
	ProjectInlier.setModelCoefficients(Coefficients);
	ProjectInlier.setModelType(pcl::SACMODEL_PLANE);
	ProjectInlier.filter(*ProjectedCloud);

	pcl::KdTreeFLANN<PointCoord> KDTree;
	KDTree.setInputCloud(ProjectedCloud);
	std::vector<int> NeighboutIndex;
	std::vector<float> NeighbourDistance;

	for (int i = 0; i < ProjectedCloud->points.size(); i++)
	{
		if (KDTree.radiusSearch(ProjectedCloud->points[i], vScale, NeighboutIndex, NeighbourDistance) > 0)
		{
			voZDirectionDestinyMap.insert(std::make_pair(i, NeighboutIndex.size()));
		}
	}

};
