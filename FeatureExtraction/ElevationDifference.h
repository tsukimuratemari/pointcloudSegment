#pragma once
#include "Common.h"
#include <pcl/kdtree/kdtree_flann.h>

class CElevationDifference
{
public:
	CElevationDifference(PointCloudT::Ptr vCloud) :m_pCloud(vCloud) {};
	virtual ~CElevationDifference() = default;

	void compute(std::unordered_map<int, float>& voElevationDifference);
	void computeBasedProjectElevation(std::unordered_map<int, float>& voElevationDifference);
	void setRadius(float vRadius) { m_Radius = vRadius; }

private:
	_NODISCARD bool __initializeKdTree();
	_NODISCARD bool __initializeProjectElevation();
	_NODISCARD bool __searchNeighbours (const PointT& vPoint,std::vector<int>& voNeighbourPointIdxs) const;
	float __getMinElevation(const std::vector<int>& vPointIdxs) const;
	float __getMinProjectElevation(const std::vector<int>& vPointIdxs) const;

private:
	PointCloudT::Ptr m_pCloud;
	pcl::search::KdTree<PointT>::Ptr m_pKdTree;
	float m_Radius=3.0f;

	std::unordered_map<int, float> m_ProjectElevation;
};