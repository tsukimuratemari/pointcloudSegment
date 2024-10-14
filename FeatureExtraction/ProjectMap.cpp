#include "pch.h"
#include "ProjectMap.h"

bool CProjectMap::isVaild() const
{
	return m_Map.size();
}

bool CProjectMap::setSize(int vWidth, int vHeight)
{
	_ASSERTE(vWidth >= 0 && vHeight >= 0 && (vWidth + vHeight) > 0);
	m_Map.resize(vWidth, vHeight);
	return true;
}

bool CProjectMap::setValueAt(const PointT& vPoint, int vRow, int vCol)
{
	_ASSERTE(__isIndexVaild(vRow, vCol));
	m_Map.coeffRef(vRow, vCol).emplace_back(vPoint);
	return true;
}

PointCloudT CProjectMap::getValueAt(int vRow, int vCol) const
{
	_ASSERTE(isVaild() && __isIndexVaild(vRow, vCol));
	return m_Map.coeffRef(vRow, vCol);
}

bool CProjectMap::__isIndexVaild(int vRow, int vCol) const
{
	return (vRow >= 0 && vRow < m_Map.rows() && vCol >= 0 && vCol < m_Map.cols());
}