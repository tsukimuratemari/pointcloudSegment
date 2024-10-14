#pragma once
#include"Common.h"

class CSuperVoxel {
public:
	CSuperVoxel(float vVoxelResolution,float vSeedResolution) :m_SupervoxelClustering(vVoxelResolution, vSeedResolution) {};
	~CSuperVoxel() = default;

	void extract(std::map <std::uint32_t, pcl::Supervoxel<PointT>::Ptr >& voSupervoxels);
	void setCloud(const PointCloudT::Ptr& vCloud) { m_pCloud = vCloud; }
	void setColorImportance(float vColorImportance) { m_ColorImportance = vColorImportance; }
	void setSpatialImportance(float vSpatialImprotance) { m_SpatialImportance = vSpatialImprotance; }
	void setNormalImportance(float vNormalImportance) { m_NormalImportance = vNormalImportance; }
	void dumpSupervoxelAdjacency(std::multimap<std::uint32_t, std::uint32_t>& voSupervoxelAdjacency)const { m_SupervoxelClustering.getSupervoxelAdjacency(voSupervoxelAdjacency); }
	void createReLabeledSuperVoxel();

private:
	PointCloudT::Ptr m_pCloud;
	float m_ColorImportance = 0.3f;
	float m_SpatialImportance = 0.4f;
	float m_NormalImportance = 1.0f;
	pcl::SupervoxelClustering<PointT> m_SupervoxelClustering;

	void __normalizePointCloud();
};