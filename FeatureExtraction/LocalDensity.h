#pragma once
#include "Common.h"
#include <pcl/segmentation/supervoxel_clustering.h>

//Q:�ڲ����ܶȾ��ȵ�����£��Ƿ�Ӧ��ʹ��radius�������Ƿ�����������ͺ�����Ч����
class CLocalDensity {
public:
	CLocalDensity() = default;
	~CLocalDensity() = default;

	void compute(const std::map <std::uint32_t, pcl::Supervoxel<PointT>::Ptr >& vSupervoxels, std::map<std::uint32_t, double>& voLocalDensitys);
};