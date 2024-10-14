#include "pch.h"
#include "ProjectMap.h"
#include "ProjectMapGenerator.h"
#include "ProjectionElevation.h"

 void CProjectionElevation::compute(std::unordered_map<int, float>& voProjectElevations)
{
	 voProjectElevations.clear();
	__generateElevationMap();
	int Count = 0;
	for (auto& Point : *m_pCloud)
	{
		voProjectElevations.emplace(Count++, __sampleProjectElevation(Point));
	}
	_ASSERTE(Count == m_pCloud->size());
}

_NODISCARD bool CProjectionElevation::setMaxAllowedDistanceWithinACluster(float vDistance)
{
	_ASSERTE(vDistance >= 0);
	m_MaxAllowedDistanceWithinACluster = vDistance;
	return true;
}

void CProjectionElevation::__generateElevationMap()
{
	_ASSERTE(m_pCloudMap && m_pCloudMap->isVaild(), "ProjectMap isn't vaild!");
	m_ElevationMap.resize(m_pCloudMap->getWidth(), m_pCloudMap->getHeight());

	for (int i = 0; i < m_ElevationMap.rows(); ++i)
	{
		for (int k = 0; k < m_ElevationMap.cols(); ++k)
		{
			auto Cloud = m_pCloudMap->getValueAt(i, k);
			m_ElevationMap.coeffRef(i, k) = __computeMaxEvelationOfClusters(Cloud);
		}
	}
}

float CProjectionElevation::__sampleProjectElevation(const PointT& vPoint) const
{
	auto Offset = m_pCloudMapGenerator->computeOffset(vPoint);
	auto Elevations = m_ElevationMap.coeffRef(Offset[0], Offset[1]);
	for (auto Elevetion : Elevations)
	{
		if (vPoint.z <= Elevetion)
			return Elevetion;
	}
	_ASSERTE(false, "All the elevations is less than point's elevation!");
	return vPoint.z;
}

bool CProjectionElevation::__initializeProjectMap(int vWidth, int vHeight)
{
	_ASSERTE(!m_pCloud->empty() && vWidth >= 0 && vHeight >= 0 && (vWidth + vHeight) > 0);
	m_pCloudMapGenerator = std::make_shared<CProjectMapGenerator>(m_pCloud);
	m_pCloudMapGenerator->generate(vWidth, vHeight);

	m_pCloudMap = std::make_shared<CProjectMap>();
	m_pCloudMapGenerator->dumpProjectMap(*m_pCloudMap);
	return true;
}

std::vector<float> CProjectionElevation::__computeMaxEvelationOfClusters(const PointCloudT& vCloud)
{
	if (vCloud.empty())return std::vector<float>();

	std::forward_list<float> Evelations;
	for (const auto& Point : vCloud)
		Evelations.emplace_front(Point.z);

	Evelations.sort();

	auto Before = Evelations.before_begin();
	auto Itr = Evelations.begin();
	auto Next = std::next(Itr);
	while (Next != Evelations.end())
	{
		if (*Next - *Itr < m_MaxAllowedDistanceWithinACluster)
			Evelations.erase_after(Before);
		else
			Before = Itr;
		Itr = Next;
		Next = std::next(Itr);
	}
	std::vector<float> Results(Evelations.begin(), Evelations.end());
	std::sort(Results.begin(), Results.end());
	return Results;
}
