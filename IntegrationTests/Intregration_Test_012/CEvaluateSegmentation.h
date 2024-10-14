#pragma once
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/features/fpfh.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointNCloudT;
typedef pcl::PointXYZL PointLT;
typedef pcl::PointCloud<PointLT> PointLCloudT;
typedef pcl::PointXYZ PointCoord;
typedef pcl::PointCloud<PointCoord> PointCloudCoord;

class CEvaluateSegmentation
{
public:
	CEvaluateSegmentation(const PointCloudT::Ptr& vCloud) :m_PointCloud(vCloud) {};
	~CEvaluateSegmentation() = default;

	void processAndSave();

protected:
	std::vector<std::vector<int>> _clusterByLabel();
	virtual std::vector<std::set<int>> _EvaluateClusterLabel(std::vector<std::set<int>>& vCluster);
	int _findMaxCountLabelInNeighbours(std::set<int>& vCluster);

	PointCloudT::Ptr m_PointCloud;
};