#include "pch.h"
#include "Supervoxel.h"

void CSuperVoxel::extract(std::map<std::uint32_t, pcl::Supervoxel<PointT>::Ptr>& voSupervoxels)
{
    if (m_pCloud->empty())return;

    __normalizePointCloud();
    m_SupervoxelClustering.setInputCloud(m_pCloud);
    m_SupervoxelClustering.setUseSingleCameraTransform(false);
    m_SupervoxelClustering.setColorImportance(m_ColorImportance);
    m_SupervoxelClustering.setNormalImportance(m_NormalImportance);
    m_SupervoxelClustering.setSpatialImportance(m_SpatialImportance);

    m_SupervoxelClustering.extract(voSupervoxels);
    pcl::console::print_info("Found %d supervoxels\n", voSupervoxels.size());
}

//visual Supervoxel
void CSuperVoxel::createReLabeledSuperVoxel()
{
    PointLCloudT::Ptr LabelPointCloud = m_SupervoxelClustering.getLabeledVoxelCloud();
    std::multimap<std::uint32_t, std::uint32_t> SupervoxelAdjacency;
    m_SupervoxelClustering.getSupervoxelAdjacency(SupervoxelAdjacency);

    std::map<std::uint32_t, std::uint32_t> LabelMapTable;
    for (std::uint32_t Label = 1; Label <= m_SupervoxelClustering.getMaxLabel(); Label++)
    {
        auto NeighbourLabelList = SupervoxelAdjacency.equal_range(Label);
        //refine label's value to the min no-zero integer that different with the neighbour's value
        std::uint32_t NewLabel = 1;
        for (auto LabelItr = NeighbourLabelList.first; LabelItr != NeighbourLabelList.second; LabelItr++)
        {
            if (LabelMapTable.find(LabelItr->second) != LabelMapTable.end())
            {
                if (NewLabel == LabelMapTable.find(LabelItr->second)->second)
                {
                    NewLabel++;
                    LabelItr = NeighbourLabelList.first;
                }
            }
            else
            {
                if (NewLabel == LabelItr->second && LabelItr->second != 0)
                {
                    NewLabel++;
                    LabelItr = NeighbourLabelList.first;
                }
            }
        }
        LabelMapTable.insert(std::make_pair(Label, NewLabel));
    }

    for (std::uint32_t i = 0; i < LabelPointCloud->points.size(); i++)
    {
        if (LabelMapTable.find(LabelPointCloud->points[i].label) != LabelMapTable.end())
        {
            LabelPointCloud->points[i].label = LabelMapTable.find(LabelPointCloud->points[i].label)->second;
        }
    }

    std::ofstream Writer("viusalSupervoxel.txt");
    for (std::size_t i = 0; i < LabelPointCloud->points.size(); i++)
    {
        Writer << LabelPointCloud->points[i].x << " ";
        Writer << LabelPointCloud->points[i].y << " ";
        Writer << LabelPointCloud->points[i].z << " ";
        Writer << LabelPointCloud->points[i].label << "\n";
    }
    Writer.close();
}

void CSuperVoxel::__normalizePointCloud()
{
    std::pair<pcl::PointXYZ, pcl::PointXYZ> MinMaxCoordinate;
    comMinMax4XYZ(m_pCloud, MinMaxCoordinate);

    float XLen = MinMaxCoordinate.second.x - MinMaxCoordinate.first.x;
    float YLen = MinMaxCoordinate.second.y - MinMaxCoordinate.first.y;
    float ZLen = MinMaxCoordinate.second.z - MinMaxCoordinate.first.z;
    float XYScale = XLen / YLen;
    float ZYScale = ZLen / YLen;
    for (std::uint32_t i = 0; i < m_pCloud->points.size(); i++)
    {
		m_pCloud->points[i].x = (m_pCloud->points[i].x - MinMaxCoordinate.first.x) / XLen * XYScale;
		m_pCloud->points[i].y = (m_pCloud->points[i].y - MinMaxCoordinate.first.y) / YLen;
		m_pCloud->points[i].z = (m_pCloud->points[i].z - MinMaxCoordinate.first.z) / ZLen * ZYScale;
    }
}
