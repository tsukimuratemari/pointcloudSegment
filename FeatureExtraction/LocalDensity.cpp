#include "pch.h"
#include "LocalDensity.h"

void CLocalDensity::compute(const std::map<std::uint32_t, pcl::Supervoxel<PointT>::Ptr>& vSupervoxels, std::map<std::uint32_t, double>& voLocalDensitys)
{
	pcl::KdTreeFLANN<PointT> KDTree;
	PointCloudT SupervoxelCloud;
	genSupervoxelCloud(vSupervoxels, SupervoxelCloud);
	KDTree.setInputCloud(std::make_shared<PointCloudT>(SupervoxelCloud));
	std::uint32_t K = 5;
	std::vector<int> PointIdxKNNSearch(K);
	std::vector<float> PointSquaredDistance(K);

	for (auto& SV : vSupervoxels)
	{
		KDTree.nearestKSearch(SV.second->centroid_, K, PointIdxKNNSearch, PointSquaredDistance);
		double LocalDensity = 0;
		for (auto SquaredDistance : PointSquaredDistance)
		{
			LocalDensity += std::sqrtl(SquaredDistance);
		}
		LocalDensity /= K;

		voLocalDensitys.insert(std::make_pair(SV.first, LocalDensity));
	}
}
