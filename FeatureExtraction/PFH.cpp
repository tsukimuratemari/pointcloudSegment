#include "pch.h"
#include "PFH.h"

void CPFH::compute(const PointCloudT::Ptr& vCloud, const std::map<std::uint32_t, pcl::Supervoxel<PointT>::Ptr>& vSupervoxels, std::map<std::uint32_t, Eigen::VectorXf>& voPFHs)
{
    pcl::PointCloud<pcl::Normal>::Ptr CloudNormals(new pcl::PointCloud<pcl::Normal>());
    pcl::search::KdTree<PointT>::Ptr Tree(new pcl::search::KdTree<PointT>());
    __estimateNormal(vCloud, Tree, CloudNormals);

    std::uint32_t K = 10;
    std::uint32_t NRSplit = 2;
    std::uint32_t PFHDimension = std::pow(NRSplit, 3);
    Eigen::VectorXf PFH(PFHDimension);
    std::vector<int> PointIdxKNNSearch;
    std::vector<float> PointSquaredDistance;
    pcl::PFHEstimation<PointT, pcl::Normal, pcl::PFHSignature125> PFHDescritor;
    
    for (auto& SV : vSupervoxels)
    {
        Tree->nearestKSearch(vSupervoxels.cbegin()->second->centroid_, K, PointIdxKNNSearch, PointSquaredDistance);
        PFHDescritor.computePointPFHSignature(*vCloud, *CloudNormals, PointIdxKNNSearch, NRSplit, PFH);
        voPFHs.insert(std::make_pair(SV.first, PFH));
    }
}

void CPFH::__estimateNormal(const PointCloudT::Ptr& vCloud, pcl::search::KdTree<PointT>::Ptr& voTree,pcl::PointCloud<pcl::Normal>::Ptr& voCloudNormals)
{
	std::uint32_t K = 5;
	pcl::NormalEstimation<PointT, pcl::Normal> NormalEstimation;
	NormalEstimation.setInputCloud(vCloud);
	NormalEstimation.setSearchMethod(voTree);
	NormalEstimation.setKSearch(K);
	NormalEstimation.compute(*voCloudNormals);
}
