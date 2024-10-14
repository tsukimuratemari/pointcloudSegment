#include "pch.h"
#include "SemanticCategory.h"

void CSemanticCategory::compute(const PointCloudT::Ptr& vOriginCloud,const std::map<std::uint32_t, pcl::Supervoxel<PointT>::Ptr>& vioSupervoxels)
{
    PointCloudT SupervoxelCloud;
    genSupervoxelCloud(vioSupervoxels, SupervoxelCloud);
    std::pair<pcl::PointXYZ, pcl::PointXYZ> VoxelCloudMinMaxCoordinate;
    comMinMax4XYZ(std::make_shared<PointCloudT>(SupervoxelCloud), VoxelCloudMinMaxCoordinate);
    float XLen4NormalizedCloud = VoxelCloudMinMaxCoordinate.second.x - VoxelCloudMinMaxCoordinate.first.x;
    float YLen4NormalizedCloud = VoxelCloudMinMaxCoordinate.second.y - VoxelCloudMinMaxCoordinate.first.y;
    float ZLen4NormalizedCloud = VoxelCloudMinMaxCoordinate.second.z - VoxelCloudMinMaxCoordinate.first.z;

    std::pair<pcl::PointXYZ, pcl::PointXYZ> OriginCloudMinMaxCoordinate;
    comMinMax4XYZ(vOriginCloud, OriginCloudMinMaxCoordinate);
    float XLen4OriginCloud = OriginCloudMinMaxCoordinate.second.x - OriginCloudMinMaxCoordinate.first.x;
    float YLen4OriginCloud = OriginCloudMinMaxCoordinate.second.y - OriginCloudMinMaxCoordinate.first.y;
    float ZLen4OriginCLoud = OriginCloudMinMaxCoordinate.second.z - OriginCloudMinMaxCoordinate.first.z;

    pcl::KdTreeFLANN<PointT> KDTree;
    KDTree.setInputCloud(vOriginCloud);
    std::uint32_t K = 5;
    std::vector<int> PointIdxKNNSearch(K);
    std::vector<float> PointSquaredDistance(K);

    for (auto& SV : vioSupervoxels)
    {
        std::map<std::uint32_t, std::uint32_t> PointLabelCount;
        for (auto& Point : SV.second->voxels_->points)
        {
            float XRelativePosition = (Point.x - VoxelCloudMinMaxCoordinate.first.x) / XLen4NormalizedCloud;
            float YRelativePosition = (Point.y - VoxelCloudMinMaxCoordinate.first.y) / YLen4NormalizedCloud;
            float ZRelativePosition = (Point.z - VoxelCloudMinMaxCoordinate.first.z) / ZLen4NormalizedCloud;

            float XPosInOriginCloud = OriginCloudMinMaxCoordinate.first.x + XRelativePosition * XLen4OriginCloud;
            float YPosInOriginCloud = OriginCloudMinMaxCoordinate.first.y + YRelativePosition * YLen4OriginCloud;
            float ZPosInOriginCloud = OriginCloudMinMaxCoordinate.first.z + ZRelativePosition * ZLen4OriginCLoud;
            PointT TMPPoint(XPosInOriginCloud, YPosInOriginCloud, ZPosInOriginCloud);
            PointIdxKNNSearch.clear();
            PointSquaredDistance.clear();
            KDTree.nearestKSearch(TMPPoint, K, PointIdxKNNSearch, PointSquaredDistance);

            std::map<std::uint32_t, std::uint32_t> NeighbourLabelCount;
            for (auto i : PointIdxKNNSearch)
            {
                __checkLabel(NeighbourLabelCount, static_cast<std::uint32_t>(vOriginCloud->points[i].a));
            }
            Point.a = static_cast<std::uint8_t>(__findMaxCountLabel(NeighbourLabelCount));
            __checkLabel(PointLabelCount, static_cast<std::uint32_t>(Point.a));
            NeighbourLabelCount.clear();
        }
        SV.second->centroid_.a = static_cast<std::uint8_t>(__findMaxCountLabel(PointLabelCount));
    }
}

void CSemanticCategory::__checkLabel(std::map<std::uint32_t, std::uint32_t>& vLabelCount, const std::uint32_t vLabel)
{
    if (vLabelCount.find(vLabel) != vLabelCount.end())
    {
        vLabelCount.at(vLabel)++;
    }
    else
    {
        vLabelCount.insert(std::make_pair(vLabel, 1));
    }
}

std::uint32_t CSemanticCategory::__findMaxCountLabel(const std::map<std::uint32_t, std::uint32_t>& vLabelCount)
{
    std::pair<std::uint32_t, std::uint32_t> MaxCountLabel(0, 0);
    for (auto& Label : vLabelCount)
    {
        if (Label.second >= MaxCountLabel.second)
            MaxCountLabel = Label;
    }
    return MaxCountLabel.first;
}
