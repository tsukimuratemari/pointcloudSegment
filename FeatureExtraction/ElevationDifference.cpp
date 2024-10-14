#include "pch.h"
#include "ElevationDifference.h"
#include "ProjectionElevation.h"

void CElevationDifference::compute(std::unordered_map<int, float>& voElevationDifference)
{

    _ASSERTE(m_pCloud && !m_pCloud->empty(), "Point cloud need to be initialized with non-empty cloud!");
    __initializeKdTree();
    voElevationDifference.reserve(m_pCloud->size());

    //#pragma omp parallel for num_threads(20)
    for (int i = 0; i < m_pCloud->size(); i++)
    {
        std::vector<int> NeighbourPointIdxs;
        __searchNeighbours(m_pCloud->points.at(i), NeighbourPointIdxs);
        float ElevationDifference = m_pCloud->points.at(i).z - __getMinElevation(NeighbourPointIdxs);
        voElevationDifference.emplace(i, ElevationDifference);
    }
}

void CElevationDifference::computeBasedProjectElevation(std::unordered_map<int, float>& voElevationDifference)
{
    _ASSERTE(m_pCloud && !m_pCloud->empty(), "Point cloud need to be initialized with non-empty cloud!");
    __initializeKdTree();
    __initializeProjectElevation();
    voElevationDifference.reserve(m_pCloud->size());

    //#pragma omp parallel for num_threads(20)
    for (int i=0;i<m_pCloud->size();i++)
    {
        std::vector<int> NeighbourPointIdxs;
        __searchNeighbours(m_pCloud->points.at(i),NeighbourPointIdxs);
        float ElevationDifference = m_ProjectElevation.at(i) - __getMinProjectElevation(NeighbourPointIdxs);
        voElevationDifference.emplace(i, ElevationDifference);
    }
}

bool CElevationDifference::__initializeKdTree()
{
    _ASSERTE(m_pCloud && !m_pCloud->empty(), "Point cloud need to be initialized with non-empty cloud!");
	if (m_pKdTree && m_pKdTree->getInputCloud() == m_pCloud)return true;
    m_pKdTree = std::make_shared<pcl::search::KdTree<PointT>>();
    m_pKdTree->setInputCloud(m_pCloud);
    return true;
}

bool CElevationDifference::__initializeProjectElevation()
{
    int Width = 256,Height=256;
    CProjectionElevation PEr(m_pCloud, Width, Height);
    PEr.compute(m_ProjectElevation);
    return true;
}

bool CElevationDifference::__searchNeighbours(const PointT& vPoint, std::vector<int>& voNeighbourPointIdxs) const
{
	_ASSERTE(m_pCloud && !m_pCloud->empty() && m_pKdTree);
    std::vector<float> TempVector;
    return m_pKdTree->radiusSearch(vPoint, m_Radius, voNeighbourPointIdxs, TempVector);
}

float CElevationDifference::__getMinElevation(const std::vector<int>& vPointIdxs) const
{
    std::vector<float> ElevationSet(vPointIdxs.size());
    for (int i = 0; i < ElevationSet.size(); i++)
    {
        ElevationSet[i] = m_pCloud->points.at(vPointIdxs[i]).z;
    }
    return *std::min_element(ElevationSet.cbegin(), ElevationSet.cend());
}

float CElevationDifference::__getMinProjectElevation(const std::vector<int>& vPointIdxs) const
{
    std::vector<float> ElevationSet(vPointIdxs.size());
    for (int i = 0; i < ElevationSet.size(); i++)
    {
        ElevationSet[i] = m_ProjectElevation.at(vPointIdxs[i]);
    }
    return *std::min_element(ElevationSet.cbegin(),ElevationSet.cend());
}

