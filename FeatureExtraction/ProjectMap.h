#pragma once
#include"Common.h"
#include<Eigen/Core>

class CProjectMap
{
public:
	CProjectMap() = default;
	~CProjectMap() = default;
	_NODISCARD bool isVaild() const;
	_NODISCARD bool setSize(int vWidth, int vHeight);
	_NODISCARD bool setValueAt(const PointT& vPoint, int vRow, int vCol);

	PointCloudT getValueAt(int vRow, int vCol) const;
	int getWidth() const { return m_Map.rows(); }
	int getHeight() const { return m_Map.cols(); }

private:
	_NODISCARD bool __isIndexVaild(int vRow, int vCol) const;


private:
	Eigen::Matrix<PointCloudT, -1, -1> m_Map;
};