#include"pch.h"
#include "FeatureBasedEigenvalue.h"

void CFeatureBasedEigenvalue::compute(const pcl::LCCPSegmentation<PointT>::SupervoxelAdjacencyList& vSupervoxelConvexityAdjacencyList, const std::map<std::uint32_t, pcl::Supervoxel<PointT>::Ptr>& vSupervoxels, std::map<std::uint32_t, SFeaturesBasedEigenvalue>& voFeaturesBasedEigenvalue)
{
    auto VertexIteratorRange = boost::vertices(vSupervoxelConvexityAdjacencyList);
    for (auto Itr = VertexIteratorRange.first; Itr != VertexIteratorRange.second; Itr++)
    {
        PointCloudT NeighbourhoodCloud;
        __genSupervoxelNeighborhoodCloud(vSupervoxelConvexityAdjacencyList, vSupervoxels, Itr, NeighbourhoodCloud);
        std::array<double, 3> Eigenvalues;
        __comCloudEigenvalue(std::make_shared<PointCloudT>(NeighbourhoodCloud), Eigenvalues);
        SFeaturesBasedEigenvalue FeatureBasedEigenvalue;
        FeatureBasedEigenvalue.comFeatures(Eigenvalues);
        if(voFeaturesBasedEigenvalue.find(static_cast<std::uint32_t>(vSupervoxelConvexityAdjacencyList[*Itr]))==voFeaturesBasedEigenvalue.end())
            voFeaturesBasedEigenvalue.insert(std::make_pair(static_cast<std::uint32_t>(vSupervoxelConvexityAdjacencyList[*Itr]), FeatureBasedEigenvalue));
    }
}

void CFeatureBasedEigenvalue::__genSupervoxelNeighborhoodCloud(const pcl::LCCPSegmentation<PointT>::SupervoxelAdjacencyList& vSupervoxelConvexityAdjacencyList, const std::map<std::uint32_t, pcl::Supervoxel<PointT>::Ptr>& vSupervoxels, const std::set<void*>::iterator vVertexItr, PointCloudT& voNeighborhoodCloud)
{
    std::uint32_t SupervoxelLabel = vSupervoxelConvexityAdjacencyList[*vVertexItr];
    voNeighborhoodCloud = *vSupervoxels.at(SupervoxelLabel)->voxels_;
    auto Neighbours = boost::adjacent_vertices(*vVertexItr, vSupervoxelConvexityAdjacencyList);
    for (auto NeighbourItr = Neighbours.first; NeighbourItr != Neighbours.second; NeighbourItr++)
    {
        pcl::LCCPSegmentation<PointT>::EdgeID Edge = boost::edge(*vVertexItr, *NeighbourItr, vSupervoxelConvexityAdjacencyList).first;
        if (vSupervoxelConvexityAdjacencyList[Edge].is_convex)
        {
            std::uint32_t NeighbourSVLabel = vSupervoxelConvexityAdjacencyList[*NeighbourItr];
            auto NeighborCloud = vSupervoxels.at(NeighbourSVLabel)->voxels_;
            voNeighborhoodCloud += *NeighborCloud;
        }
    }
}

void CFeatureBasedEigenvalue::__comCloudEigenvalue(const PointCloudT::Ptr& vCloud, std::array<double, 3>& voEigenvalues)
{
    Eigen::Matrix3f CovarianceMatrix;
    Eigen::Vector4f Centeroid;
    Eigen::Vector3d Eigenvalues;
    pcl::compute3DCentroid(*vCloud, Centeroid);
    //Q:该不该归一化协方差矩阵呢？
    pcl::computeCovarianceMatrix(*vCloud, Centeroid, CovarianceMatrix);
    pcl::eigen33(CovarianceMatrix, Eigenvalues);

    //eigenvalues in descending order
    voEigenvalues[0] = Eigenvalues.x();
    voEigenvalues[1] = Eigenvalues.y();
    voEigenvalues[2] = Eigenvalues.z();
    std::sort(voEigenvalues.begin(), voEigenvalues.end(), [](double X, double Y) {return X > Y; });
}
