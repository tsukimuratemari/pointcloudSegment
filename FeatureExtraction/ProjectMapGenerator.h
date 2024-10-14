#pragma once
#include"Common.h"
#include "ProjectMap.h"
#include<Eigen/Core>

class CProjectMapGenerator
{
public:
	CProjectMapGenerator(const PointCloudT::Ptr& vCloud) :m_pCloud(vCloud) {};
	~CProjectMapGenerator() = default;

	void generate(int vWidth, int vHeight);
	void dumpProjectMap(CProjectMap& voMap) { voMap = m_Map; }
	Eigen::Vector2i computeOffset(const PointT& vPoint) const;
private:

private:
	CProjectMap m_Map;
	PointCloudT::Ptr m_pCloud;
	SAABB m_Box;
};