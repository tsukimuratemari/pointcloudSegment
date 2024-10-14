#include "pch.h"
#include "Direction.h"

void CSupervoxelDirection::compute(const std::map<std::uint32_t, pcl::Supervoxel<PointT>::Ptr>& vSupervoxels, std::map<std::uint32_t, double>& voSVDirections)
{
	for (auto& SV : vSupervoxels)
	{
		double SVDirection;
		SVDirection= SV.second->normal_.normal_z /
			(std::sqrtl(std::powl(SV.second->normal_.normal_x, 2) + std::powl(SV.second->normal_.normal_y, 2) + std::powl(SV.second->normal_.normal_z, 2)));
		voSVDirections.insert(std::make_pair(SV.first,SVDirection));
	}
}
