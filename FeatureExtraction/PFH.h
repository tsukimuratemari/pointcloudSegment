#pragma once
#include "Common.h"
#include <pcl/segmentation/supervoxel_clustering.h>

class CPFH {
public:
	CPFH() = default;
	~CPFH() = default;

	void compute(const PointCloudT::Ptr& vCloud,const std::map <std::uint32_t, pcl::Supervoxel<PointT>::Ptr >& vSupervoxels,
		std::map<std::uint32_t,Eigen::VectorXf>& voPFHs);

private:
	void __estimateNormal(const PointCloudT::Ptr& vCloud, pcl::search::KdTree<PointT>::Ptr& voTree, pcl::PointCloud<pcl::Normal>::Ptr& voCloudNormals);

};