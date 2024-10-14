#pragma once
#include "Common.h"
#include <pcl/segmentation/supervoxel_clustering.h>

class CRelativeElevation {
public:
	CRelativeElevation(const PointCloudT::Ptr& vOriginCloud, std::uint32_t vCellScale, std::uint32_t vKernelScale)
		:m_OriginCloud(vOriginCloud), m_CellScale(vCellScale), m_KernelScale(vKernelScale) {};
	~CRelativeElevation() = default;

	void compute(const std::map <std::uint32_t, pcl::Supervoxel<PointT>::Ptr >& vSupervoxels, std::map<std::uint32_t, float>& voRelativeElevations);
	/*float comRelativeElevation4Supervoxel(pcl::Supervoxel<PointT>::Ptr vSupervoxel);*/

#ifdef _UNIT_TEST
	void divideCloud2Cells(std::map<std::uint32_t, SCell>& vCells) { __divideCloud2Cells(vCells); }
	void getCellNeighbourIds(const std::map<std::uint32_t, SCell>& vCells, const SCell& vCell, std::vector<std::uint32_t>& vNeighbourIds)
	{
		__getCellNeighbourIds(vCells, vCell, vNeighbourIds);
	}
	void comAveElevationAndAbsGroundValue(const std::map<std::uint32_t, SCell>& vCells, std::map<std::uint32_t, float>& vAveElevations, std::map<std::uint32_t, float>& vAbsoluteGroundValues)
	{
		__comAveElevationAndAbsGroundValue(vCells, vAveElevations, vAbsoluteGroundValues);
	}
	bool IsSatisfyGaussianDistribution(const SCell& vCell, const std::map<std::uint32_t, float>& vAveElevations)
	{
		return __IsSatisfyGaussianDistribution(vCell, vAveElevations);
	}

	CRelativeElevation()=default;
	void setCloud(const PointCloudT::Ptr& vOriginCloud) { m_OriginCloud = vOriginCloud; }
	void setCellScale(std::uint32_t vCellScale) { m_CellScale = vCellScale; }
	void setKernelScale(std::uint32_t vKernelScale) { m_KernelScale = vKernelScale; }
#endif // _UNIT_TEST


private:
	PointCloudT::Ptr m_OriginCloud;
	std::uint32_t m_CellScale;
	std::uint32_t m_KernelScale;

	void __divideCloud2Cells(std::map<std::uint32_t, SCell>& voCells);
	void __getCellNeighbourIds(const std::map<std::uint32_t, SCell>& vCells,const SCell& vCell, std::vector<std::uint32_t>& voNeighbourIds);
	void __comAveElevationAndAbsGroundValue(const std::map<std::uint32_t, SCell>& vCells, std::map<std::uint32_t,float>& voAveElevations,std::map<std::uint32_t,float>& voAbsoluteGroundValues);
	bool __IsSatisfyGaussianDistribution(const SCell& vCell,const std::map<std::uint32_t, float>& vAveElevations);
	void __comGroundValue4Cells(std::map<std::uint32_t, float>& voGroundValue4Cells);

};