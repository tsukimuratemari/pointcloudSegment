#pragma once
#include "Common.h"
#include <pcl/segmentation/supervoxel_clustering.h>

class CLAB {
public:
	CLAB() = default;
	CLAB(PointCloudT::Ptr vCloud) :m_pCloud(vCloud) {};
	~CLAB() = default;

	void compute(std::unordered_map<int, std::array<float, 3>>& voLABs);
	void computeBasedSV(const std::map <std::uint32_t, pcl::Supervoxel<PointT>::Ptr >& vSupervoxels, std::map<std::uint32_t, std::array<float, 3>>& voLABs);

private:
	void __convertRGB2LAB(std::array<float, 3> vRGB, std::array<float, 3>& voLAB);
	void __convertRGB2XYZ(const std::array<float, 3>& vRGB, std::array<float, 3>& voXYZ);
	void __convertXYZ2LAB(std::array<float, 3>& vXYZ, std::array<float, 3>& voLAB);
	void __correctingGamma(std::array<float, 3>& vioRGB);

private:
	PointCloudT::Ptr m_pCloud;
	
};

