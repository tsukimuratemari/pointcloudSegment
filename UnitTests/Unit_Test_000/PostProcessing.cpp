#include "pch.h"
#include "PostProcessing.h"

void CVehiclesPostProcessing::process()
{
    std::vector<std::set<std::uint32_t>> VehicleClusters;
    _clustering(VehicleClusters);
    for (auto& Cluster : VehicleClusters)
    {
        auto Neighbors = _findClusterNeighbourHood(Cluster);
        if (!__ruleByVechicleArea(Cluster) || !__ruleByNeighbour(Neighbors))
            _changeLabelBasedMaxCountLabelInNeighbors(Cluster, Neighbors);
    }

    auto computeSVsHeight = [&](const std::set<std::uint32_t> vSVs)
    {
        float PointHeightSum = 0;
        std::uint32_t PointCount = 0;
        for (auto SVSerialNum : vSVs)
        {
            auto SVPoints = m_Points.equal_range(SVSerialNum);
            for (auto Itr = SVPoints.first; Itr != SVPoints.second; Itr++)
            {
                PointHeightSum += Itr->second[m_HeightColumnIndex];
                PointCount++;
            }
                
        }
        return PointCount == 0 ? 0.0f : (PointHeightSum / PointCount);
    };

    VehicleClusters.clear();
    _clustering(VehicleClusters);
    std::set<std::uint32_t> VehicleSVs;
    for (auto& Cluster : VehicleClusters)
        VehicleSVs.merge(Cluster);

    m_SemanticLabel = ESemanticCategory::Ground;
    std::vector<std::set<std::uint32_t>> GroundClusters;
    _clustering(GroundClusters);
    std::set<std::uint32_t> GroundSVs;
    for (auto& Cluster : GroundClusters)
        GroundSVs.merge(Cluster);

    float GroundHeight = computeSVsHeight(GroundSVs);
    std::cout << "GroundHeight:" << GroundHeight << std::endl;
    for (auto SV : VehicleSVs)
    {
        std::set<std::uint32_t> SVSet{ SV };
        if (computeSVsHeight(SVSet) - GroundHeight <= m_MinHeightDifference)
            m_Supervoxels.at(SV)[m_SemanticLabelColumnIndex] = static_cast<double>(ESemanticCategory::Ground);
    }
    _convertMap2DoubleVector();
}

bool CVehiclesPostProcessing::__ruleByVechicleArea(const std::set<std::uint32_t>& vVehicleCluster) const
{
    return _computeArea(vVehicleCluster) >= m_MinVechicleArea;
}

//at least one neighbour is ground
bool CVehiclesPostProcessing::__ruleByNeighbour(const std::set<std::uint32_t>& vNeighbors) const
{
    for (auto Neighbor : vNeighbors)
    {
        ESemanticCategory NeighborSemanticLabel = static_cast<ESemanticCategory>(m_Supervoxels.at(Neighbor)[m_SemanticLabelColumnIndex]);
        if (NeighborSemanticLabel == ESemanticCategory::Ground)
            return true;
    }
    return false;
}

bool CVehiclesPostProcessing::__ruleByRelativeHeight(const std::uint32_t vVehicleSV, std::set<std::uint32_t> vNeighbors) const
{
    auto computeSVsHeight = [&](const std::set<std::uint32_t> vSVs)
    {
        float PointHeightSum = 0;
        std::uint32_t PointCount = 0;
        for (auto SVSerialNum : vSVs)
        {
            auto SVPoints = m_Points.equal_range(SVSerialNum);
            for (auto Itr = SVPoints.first; Itr != SVPoints.second; Itr++)
            {
                PointHeightSum += Itr->second[m_HeightColumnIndex];
                PointCount++;
            }
               
        }
        return PointCount == 0 ? 0.0f : (PointHeightSum / PointCount);
    };

    std::set<std::uint32_t> VehicleSV{vVehicleSV};
    auto VehicleHeight = computeSVsHeight(VehicleSV);

    for (auto Itr=vNeighbors.begin();Itr!=vNeighbors.end();)
    {
        auto NeighborLabel = static_cast<ESemanticCategory>(m_Supervoxels.at(*Itr)[m_SemanticLabelColumnIndex]);
        if (NeighborLabel != ESemanticCategory::Ground)
            Itr = vNeighbors.erase(Itr);
        else
            ++Itr;
    }
    auto NeighborsHeight = computeSVsHeight(vNeighbors);
    return VehicleHeight - NeighborsHeight > m_MinHeightDifference;
}

std::set<std::uint32_t> CPostProcessing::_findClusterNeighbourHood(const std::set<std::uint32_t>& vCluster) const
{
    std::set<std::uint32_t> Neighbors;
    for (auto SV : vCluster)
    {
        auto NeighborRange = m_SVAdjacencyList.equal_range(SV);
        for (auto Itr = NeighborRange.first; Itr != NeighborRange.second; Itr++)
            if (static_cast<ESemanticCategory>(m_Supervoxels.at(Itr->second)[m_SemanticLabelColumnIndex]) != m_SemanticLabel)
                Neighbors.insert(Itr->second);
    }
    return Neighbors;
}

ESemanticCategory CPostProcessing::_findMaxCountLabelOfNeighbourHood(const std::set<std::uint32_t>& vNeighbourHood) const
{
    if (vNeighbourHood.empty())return m_SemanticLabel;
    std::map<ESemanticCategory, std::uint32_t> LabelCountStatistics;
    for (auto Neighbour : vNeighbourHood)
    {
        ESemanticCategory NeighborLabel = static_cast<ESemanticCategory>(m_Supervoxels.at(Neighbour)[m_SemanticLabelColumnIndex]);
        if (LabelCountStatistics.contains(NeighborLabel))
            LabelCountStatistics.at(NeighborLabel)++;
        else
            LabelCountStatistics.insert(std::make_pair(NeighborLabel, 1));
    }

    auto sortValueByDescendingOrder = [](const std::pair<ESemanticCategory, std::uint32_t>& vLhs, const std::pair<ESemanticCategory, std::uint32_t>& vRhs) {
        return vLhs.second > vRhs.second;
    };
    std::vector<std::pair<ESemanticCategory, std::uint32_t>> LabelCountInDescendingOrder(LabelCountStatistics.begin(), LabelCountStatistics.end());
    std::sort(LabelCountInDescendingOrder.begin(), LabelCountInDescendingOrder.end(), sortValueByDescendingOrder);
    return LabelCountInDescendingOrder.front().first;
}

float CPostProcessing::_computeClusterHeight(const std::set<std::uint32_t>& vCluster) const
{
    float SumHeight = 0.0f;
    std::uint32_t PointCount = 0;
    for (auto SV : vCluster)
    {
        auto Points = m_Points.equal_range(SV);
        for (auto Itr = Points.first; Itr != Points.second; Itr++)
            SumHeight += Itr->second[m_HeightColumnIndex];
        PointCount += m_Points.count(SV);
    }
	return PointCount == 0 ? SumHeight : (SumHeight / PointCount);
}

std::uint32_t CPostProcessing::_computeArea(const std::set<std::uint32_t>& vCluster) const
{
    std::uint32_t PointCount = 0;
    for (auto SV : vCluster)
        PointCount += m_Points.count(SV);
    return PointCount;
}

void CPostProcessing::process()
{
}

void CPostProcessing::_convertDataFormat2Map(const std::vector<std::vector<double>>& vSVs)
{
    _ASSERTE(!vSVs.empty());
    m_Supervoxels.clear();
    for (auto& SV : vSVs)
    {
        std::uint32_t SVSerialNum = SV[m_SupervoxelSerialNumColumnIndex];
        m_Supervoxels.insert(std::make_pair(SVSerialNum, SV));
    }
}

void CPostProcessing::_convertDataFormat2MultiMap(const std::vector<std::vector<double>>& vSVs)
{
    _ASSERTE(!vSVs.empty());
    for (auto& SV : vSVs)
    {
        std::uint32_t SVSerialNum = SV[m_SupervoxelSerialNumColumnIndex];
        m_Points.insert(std::make_pair(SVSerialNum, SV));
    }
}

void CPostProcessing::_convertMap2DoubleVector()
{
    for (auto& SV : m_OriginFormatSVs)
    {
        SV[m_SemanticLabelColumnIndex] = m_Supervoxels.at(SV[m_SupervoxelSerialNumColumnIndex])[m_SemanticLabelColumnIndex];
    }
}

void CPostProcessing::_changeLabelBasedMaxCountLabelInNeighbors(const std::set<std::uint32_t>& vCluster, const std::set<std::uint32_t>& vNeighbors)
{
    ESemanticCategory MaxCountLabelInNeighbor = _findMaxCountLabelOfNeighbourHood(vNeighbors);
    for (auto SV : vCluster)
        m_Supervoxels.at(SV)[m_SemanticLabelColumnIndex] = static_cast<double>(MaxCountLabelInNeighbor);
}

void CPostProcessing::_clustering(std::vector<std::set<std::uint32_t>>& voClusters) const
{
    _ASSERTE(!m_Supervoxels.empty() && !m_SVAdjacencyList.empty());
    auto NonClusterSVs = m_Supervoxels;
    std::set<std::uint32_t> Cluster;
    std::vector<std::uint32_t> Seeds;

    auto __grow = [&](const std::uint32_t vSVSerialNum) {
        Seeds.emplace_back(vSVSerialNum);
        Cluster.clear();
        while (!Seeds.empty())
        {
            auto LastSeed = Seeds.back();
            Seeds.pop_back();
            Cluster.insert(LastSeed);
            auto Neighbours = m_SVAdjacencyList.equal_range(LastSeed);
            for (auto Itr = Neighbours.first; Itr != Neighbours.second; Itr++)
            {
				if (NonClusterSVs.contains(Itr->second) && static_cast<ESemanticCategory>(NonClusterSVs.at(Itr->second)[m_SemanticLabelColumnIndex]) == m_SemanticLabel)
                {
                    Seeds.emplace_back(Itr->second);
                    NonClusterSVs.erase(Itr->second);
                }  
            }
        }
    };

    for (auto& SV : m_Supervoxels)
    {
        if (static_cast<ESemanticCategory>(SV.second[m_SemanticLabelColumnIndex]) == m_SemanticLabel && NonClusterSVs.contains(SV.first))
        {
            __grow(SV.first);
            voClusters.emplace_back(Cluster);
        }
    }
}

void CGroundPostProcessing::process()
{
    std::vector<std::set<std::uint32_t>> GroundClusters;
    _clustering(GroundClusters);
    for (auto& Cluster : GroundClusters)
    {
        auto Neighbors = _findClusterNeighbourHood(Cluster);
        if (!__ruleByGroundArea(Cluster)||!__ruleByNeighbourHeight(Cluster,Neighbors))
            _changeLabelBasedMaxCountLabelInNeighbors(Cluster, Neighbors);
    }
    _convertMap2DoubleVector();
}

bool CGroundPostProcessing::__ruleByGroundArea(const std::set<std::uint32_t>& vGroundCluster) const
{
    return _computeArea(vGroundCluster) >= m_MinGroundArea;
}

bool CGroundPostProcessing::__ruleByNeighbourHeight(const std::set<std::uint32_t>& vGroundCluster,const std::set<std::uint32_t>& vNeighbors) const
{
    return _computeClusterHeight(vGroundCluster) <= _computeClusterHeight(vNeighbors);
}

void CBuildingPostProcessing::process()
{
    std::vector<std::set<std::uint32_t>> BuildingClusters;
    _clustering(BuildingClusters);
    for (auto& Cluster : BuildingClusters)
    {
        auto Neighbors = _findClusterNeighbourHood(Cluster);
        if (!__ruleByBuildingArea(Cluster))
            _changeLabelBasedMaxCountLabelInNeighbors(Cluster, Neighbors);
        else
        {
            auto NeighborsExceptTree = Neighbors;
            for (auto Itr = NeighborsExceptTree.begin(); Itr != NeighborsExceptTree.end();)
            {
                if (static_cast<ESemanticCategory>(m_Supervoxels.at(*Itr)[m_SemanticLabelColumnIndex]) == ESemanticCategory::Tree)
                    Itr = NeighborsExceptTree.erase(Itr);
                else
                    ++Itr;
            }
            if (!__ruleByNeighborHeight(Cluster, NeighborsExceptTree))
                _changeLabelBasedMaxCountLabelInNeighbors(Cluster, NeighborsExceptTree);
        }
    }
    _convertMap2DoubleVector();
}

bool CBuildingPostProcessing::__ruleByBuildingArea(const std::set<std::uint32_t>& vBuildingCluster) const
{
    return _computeArea(vBuildingCluster) >= m_MinBuildingArea;
}

//************************************************
//FUNCTION:the height of building should be higher than neighbors except for trees;
bool CBuildingPostProcessing::__ruleByNeighborHeight(const std::set<std::uint32_t>& vBuildingCluster,const std::set<std::uint32_t>& vNeighborsExceptTree) const
{
    return _computeClusterHeight(vBuildingCluster) - _computeClusterHeight(vNeighborsExceptTree) > m_HeightDifferenceThreshold;
}

void CTreePostProcessing::process()
{
    std::vector<std::set<std::uint32_t>> TreeClusters;
    _clustering(TreeClusters);
    for (auto& Cluster : TreeClusters)
    {
        if (!__ruleByTreeArea(Cluster))
        {
            auto Neighbors = _findClusterNeighbourHood(Cluster);
            _changeLabelBasedMaxCountLabelInNeighbors(Cluster, Neighbors);
        }
    }
    _convertMap2DoubleVector();
}

bool CTreePostProcessing::__ruleByTreeArea(const std::set<std::uint32_t>& vTreeCluster) const
{
    return _computeArea(vTreeCluster) >= m_MinTreeArea;
}

bool CTreePostProcessing::__ruleByNormalDifference(const std::set<std::uint32_t>& vTreeCluster) const
{
    return false;
}
