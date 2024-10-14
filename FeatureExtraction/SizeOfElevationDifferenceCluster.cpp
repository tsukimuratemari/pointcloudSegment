#include"pch.h"
#include"SizeOfElevationDifferenceCluster.h"

void CSizeOfElevationDifferenceCluster::compute(std::unordered_map<PointSerialNumber, FeatureValue>& voSizeOfElevationDifferenceCluster)
{
	std::unordered_multimap<ClusterSerialNumber, PointSerialNumber> ClusterMapPoint;
	std::unordered_map<PointSerialNumber, ClusterSerialNumber> PointMapCluster;
	__clusterByElevationDifference(ClusterMapPoint, PointMapCluster);

	std::unordered_map<ClusterSerialNumber, FeatureValue> ClusterMapFeatureValue;
	__computeClusterFeatureValues(ClusterMapPoint, ClusterMapFeatureValue);

	for (int i = 0; i < m_pCloud->size(); i++)
	{
		_HIVE_EARLY_EXIT(!PointMapCluster.contains(i), "PointMapCluster don't contain this point!");
		_HIVE_EARLY_EXIT(!ClusterMapFeatureValue.at(PointMapCluster.at(i)), "ClusterMapFeatureValue don't contain this cluster!");
		FeatureValue Value = ClusterMapFeatureValue.at(PointMapCluster.at(i));
		voSizeOfElevationDifferenceCluster.emplace(i, Value);
	}
}

void CSizeOfElevationDifferenceCluster::__clusterByElevationDifference(std::unordered_multimap<ClusterSerialNumber, PointSerialNumber>& voClusters,
	std::unordered_map<PointSerialNumber, ClusterSerialNumber>& voPoints)
{
	_HIVE_EARLY_EXIT(m_pCloud->empty(), "Point cloud is empty!");
	_HIVE_EARLY_EXIT(m_ElevationDifferences.empty(), "ElevationDifference data is empty!");
	_HIVE_EARLY_EXIT(m_pCloud->size() != m_ElevationDifferences.size(), "The size of cloud and ElevationDifference isn't equal!");
	
	__initializeKdTree();

	int CurrentSerialNumber = 0;
	auto __growRegion = [&](const int vPointIndex)
	{
		if (voPoints.contains(vPointIndex))return;
		std::unordered_set<PointSerialNumber> Seeds;
		std::vector<int> NeighbourPointIndexs;
		Seeds.insert(vPointIndex);

		while (!Seeds.empty())
		{
			int Seed = *Seeds.begin();
			voClusters.emplace(CurrentSerialNumber, Seed);
			voPoints.emplace(Seed, CurrentSerialNumber);
			Seeds.erase(Seeds.begin());
			NeighbourPointIndexs.clear();
			__searchNeighbours(m_pCloud->points.at(Seed), NeighbourPointIndexs);
			if (!__checkNeighbours(NeighbourPointIndexs))continue;
			for (auto NeighbourPointIndex : NeighbourPointIndexs)
			{
				if (!voPoints.contains(NeighbourPointIndex))
				{
					auto TheDifferenceOfElevationDifferences = m_ElevationDifferences.at(Seed) - m_ElevationDifferences.at(NeighbourPointIndex);
					if (std::abs(TheDifferenceOfElevationDifferences) < m_DifferenceThreshold)
					{
						Seeds.insert(NeighbourPointIndex);
					}
				}
			}

		}
		CurrentSerialNumber++;
	};

	for (int i = 0; i < m_pCloud->size(); i++)
	{
		__growRegion(i);
	}
}

void CSizeOfElevationDifferenceCluster::__computeClusterFeatureValues(const std::unordered_multimap<ClusterSerialNumber, PointSerialNumber>& vClusters, 
	std::unordered_map<ClusterSerialNumber, FeatureValue>& voClusterMapFeatureValue)
{
	if (vClusters.empty())return;

	//NOTE:the serial number of cluster is begin from 0;
	ClusterSerialNumber CurrentSerialNumber = 0;
	while (vClusters.contains(CurrentSerialNumber))
	{
		voClusterMapFeatureValue.emplace(CurrentSerialNumber, vClusters.count(CurrentSerialNumber));
		CurrentSerialNumber++;
	}
}

bool CSizeOfElevationDifferenceCluster::__initializeKdTree()
{
	_HIVE_EARLY_RETURN(m_pCloud && m_pCloud->empty(), "Point cloud need to be initialized with non-empty cloud!", false);
	if (m_pKdTree && m_pKdTree->getInputCloud() == m_pCloud)return true;
	m_pKdTree = std::make_shared<pcl::search::KdTree<PointT>>();
	m_pKdTree->setInputCloud(m_pCloud);
	return true;
}

bool CSizeOfElevationDifferenceCluster::__searchNeighbours(const PointT& vPoint, std::vector<int>& voNeighbourPointIdxs) const
{
	_HIVE_EARLY_RETURN(m_pKdTree && m_pKdTree->getInputCloud()->empty(), "KDTree isn't be initialized!", false);
	std::vector<float> TempVector;
	return m_pKdTree->nearestKSearch(vPoint, m_NeighbourNum, voNeighbourPointIdxs, TempVector);
}

bool CSizeOfElevationDifferenceCluster::__checkNeighbours(const std::vector<int>& vNeighbourPointIndexs) const
{
	for (auto Point : vNeighbourPointIndexs)
	{
		if (m_ElevationDifferences.at(Point) < m_GroundMaxThreshold)return false;
	}
	return true;
}
