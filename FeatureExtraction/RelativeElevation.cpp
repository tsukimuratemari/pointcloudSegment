#include "pch.h"
#include "RelativeElevation.h"

void CRelativeElevation::compute(const std::map<std::uint32_t, pcl::Supervoxel<PointT>::Ptr>& vSupervoxels, std::map<std::uint32_t, float>& voRelativeElevations)
{
    std::map<std::uint32_t, float> GroundValue4Cells;
    __comGroundValue4Cells(GroundValue4Cells);

    std::pair<pcl::PointXYZ, pcl::PointXYZ> MinMaxCoordinate;
    comMinMax4XYZ(m_OriginCloud, MinMaxCoordinate);
    float XCellLen = (MinMaxCoordinate.second.x - MinMaxCoordinate.first.x) / m_CellScale;
    float YCellLen = (MinMaxCoordinate.second.y - MinMaxCoordinate.first.y) / m_CellScale;

    for (auto& SV : vSupervoxels)
    {
        std::uint32_t XIndex = static_cast<std::uint32_t>((SV.second->centroid_.x - MinMaxCoordinate.first.x) / XCellLen);
        std::uint32_t YIndex = static_cast<std::uint32_t>((SV.second->centroid_.y - MinMaxCoordinate.first.y) / YCellLen);
        if (XIndex >= m_CellScale)XIndex = m_CellScale - 1;
        if (YIndex >= m_CellScale)YIndex = m_CellScale - 1;
        std::uint32_t CellId = XIndex + YIndex * m_CellScale;

        float RelativeElevation;
        if (GroundValue4Cells.contains(CellId))
        {
            RelativeElevation = SV.second->centroid_.z - GroundValue4Cells.at(CellId);
        }
        else
            RelativeElevation = SV.second->centroid_.z;
        voRelativeElevations.insert(std::make_pair(SV.first,RelativeElevation));
    }
}

//float CRelativeElevation::comRelativeElevation4Supervoxel(pcl::Supervoxel<PointT>::Ptr vSupervoxel)
//{
//    std::map<std::uint32_t, float> GroundValue4Cells;
//    __comGroundValue4Cells(GroundValue4Cells);
//
//    std::pair<pcl::PointXYZ, pcl::PointXYZ> MinMaxCoordinate;
//    comMinMax4XYZ(m_OriginCloud, MinMaxCoordinate);
//    float XCellLen = (MinMaxCoordinate.second.x - MinMaxCoordinate.first.x) / m_CellScale;
//    float YCellLen = (MinMaxCoordinate.second.y - MinMaxCoordinate.first.y) / m_CellScale;
//    std::uint32_t XIndex = static_cast<std::uint32_t>((vSupervoxel->centroid_.x - MinMaxCoordinate.first.x) / XCellLen);
//    std::uint32_t YIndex = static_cast<std::uint32_t>((vSupervoxel->centroid_.y - MinMaxCoordinate.first.y) / YCellLen);
//    if (XIndex >= m_CellScale)XIndex = m_CellScale - 1;
//    if (YIndex >= m_CellScale)YIndex = m_CellScale - 1;
//    std::uint32_t CellId = XIndex + YIndex * m_CellScale;
//
//    return vSupervoxel->centroid_.z - GroundValue4Cells.at(CellId);
//}

void CRelativeElevation::__divideCloud2Cells(std::map<std::uint32_t, SCell>& voCells)
{
    std::pair<pcl::PointXYZ, pcl::PointXYZ> MinMaxCoordinate;
    comMinMax4XYZ(m_OriginCloud, MinMaxCoordinate);
    float XCellLen = (MinMaxCoordinate.second.x - MinMaxCoordinate.first.x) / m_CellScale;
    float YCellLen = (MinMaxCoordinate.second.y - MinMaxCoordinate.first.y) / m_CellScale;
    for (auto& Point : *m_OriginCloud)
    {
        std::uint32_t XIndex = static_cast<std::uint32_t>((Point.x - MinMaxCoordinate.first.x) / XCellLen);
        std::uint32_t YIndex = static_cast<std::uint32_t>((Point.y - MinMaxCoordinate.first.y) / YCellLen);
        if (XIndex >= m_CellScale)XIndex = m_CellScale - 1;
        if (YIndex >= m_CellScale)YIndex = m_CellScale - 1;
        std::uint32_t Id = XIndex + YIndex * m_CellScale;
        if (voCells.find(Id) != voCells.end())
        {
            voCells.find(Id)->second._Id = Id;
            voCells.find(Id)->second._cloud.push_back(Point);
        }
        else
        {
            voCells.insert(std::make_pair(Id, SCell()));
            voCells.find(Id)->second._Id = Id;
            voCells.find(Id)->second._cloud.push_back(Point);
        }
    }
}

void CRelativeElevation::__getCellNeighbourIds(const std::map<std::uint32_t, SCell>& vCells,const SCell& vCell, std::vector<std::uint32_t>& voNeighbourIds)
{
    int XIndex = vCell._Id % m_CellScale;
    int YIndex = vCell._Id / m_CellScale;
    int HalfKernelScale = m_KernelScale / 2;

    for (int i = XIndex - HalfKernelScale; i <= XIndex + HalfKernelScale; i++)
    {
        for (int k = YIndex - HalfKernelScale; k <= YIndex + HalfKernelScale; k++)
        {
            if (i < 0 || i >= m_CellScale || k < 0 || k >= m_CellScale)continue;
            if (vCells.find(i + k * m_CellScale) != vCells.end())
            {
                voNeighbourIds.push_back(i + k * m_CellScale);
            }
        }
    }
}

void CRelativeElevation::__comAveElevationAndAbsGroundValue(const std::map<std::uint32_t, SCell>& vCells, std::map<std::uint32_t, float>& voAveElevations, std::map<std::uint32_t, float>& voAbsoluteGroundValues)
{
    for (auto CellItr = vCells.begin(); CellItr != vCells.end(); CellItr++)
    {
        float AverageElevation = 0;
        float AbsoluteGroundValue = FLT_MAX;
        for (auto PointItr = CellItr->second._cloud.begin(); PointItr != CellItr->second._cloud.end(); PointItr++)
        {
            AverageElevation += PointItr->z;
            AbsoluteGroundValue = AbsoluteGroundValue < PointItr->z ? AbsoluteGroundValue : PointItr->z;
        }
        voAveElevations.insert(std::make_pair(CellItr->first, AverageElevation / CellItr->second._cloud.size()));
        voAbsoluteGroundValues.insert(std::make_pair(CellItr->first, AbsoluteGroundValue));
    }
}

bool CRelativeElevation::__IsSatisfyGaussianDistribution(const SCell& vCell, const std::map<std::uint32_t, float>& vAveElevations)
{
    float StandardDeviation = 0;
    float CellAveElevation = vAveElevations.find(vCell._Id)->second;
    for (auto PointItr = vCell._cloud.begin(); PointItr != vCell._cloud.end(); PointItr++)
    {
        StandardDeviation += std::powf(PointItr->z - CellAveElevation, 2);
    }
    StandardDeviation = std::sqrtf(StandardDeviation / vCell._cloud.size());

    std::uint32_t NumLocateinOneStandardDeviation = 0;
    for (auto PointItr = vCell._cloud.begin(); PointItr != vCell._cloud.end(); PointItr++)
    {
        if ((PointItr->z <= CellAveElevation + StandardDeviation) && (PointItr->z >= CellAveElevation - StandardDeviation))
        {
            NumLocateinOneStandardDeviation++;
        }
    }

    float ActulPercentage = static_cast<float>(NumLocateinOneStandardDeviation) / vCell._cloud.size();
    float OneSTDPercentage = 0.6826;
    float Error = 0.03;
    return (ActulPercentage <= (OneSTDPercentage + Error)) && (ActulPercentage >= (OneSTDPercentage - Error));
}

void CRelativeElevation::__comGroundValue4Cells(std::map<std::uint32_t, float>& voGroundValue4Cells)
{
    std::map<std::uint32_t, SCell> Cells;
    __divideCloud2Cells(Cells);

    std::map<std::uint32_t, float> AveElevations;
    std::map<std::uint32_t, float> AbsGroundValues;
    __comAveElevationAndAbsGroundValue(Cells, AveElevations, AbsGroundValues);

    for (auto CellItr = Cells.cbegin(); CellItr != Cells.cend(); CellItr++)
    {
        if (__IsSatisfyGaussianDistribution(CellItr->second, AveElevations))
        {
            voGroundValue4Cells.insert(std::make_pair(CellItr->first,AbsGroundValues.at(CellItr->first)));
        }
        else
        {
            std::vector<std::uint32_t> CellNeighbourIds;
            __getCellNeighbourIds(Cells, CellItr->second, CellNeighbourIds);
            float GroundValue = 0;
            for (auto IdItr = CellNeighbourIds.cbegin(); IdItr != CellNeighbourIds.cend(); IdItr++)
            {
                GroundValue += AbsGroundValues.at(*IdItr);
            }
            if (CellNeighbourIds.size() != 0)GroundValue /= CellNeighbourIds.size();
            voGroundValue4Cells.insert(std::make_pair(CellItr->first, GroundValue));
        }
    }
}
