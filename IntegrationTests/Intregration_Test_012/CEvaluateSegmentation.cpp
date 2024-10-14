#include "pch.h"
#include "CEvaluateSegmentation.h"

void CEvaluateSegmentation::processAndSave()
{
}

std::vector<std::vector<int>> CEvaluateSegmentation::_clusterByLabel()
{
	std::vector<std::vector<int>> Clusters;
	pcl::KdTreeFLANN<PointT> KDTree;
	KDTree.setInputCloud(m_PointCloud);
	int K = 10;

	std::set<int> UnClusteredPointSet;
	for (int i = 0; i < m_PointCloud->size(); i++)UnClusteredPointSet.emplace(i);
	//search all cluster and save in a vector
	for (int i = 0; i < m_PointCloud->size(); i++)
	{
		if (UnClusteredPointSet.find(i) != UnClusteredPointSet.end())
		{
			//search a cluster
			std::deque<int> SeedPointDeque;
			std::vector<int> Cluster;
			SeedPointDeque.push_back(i);
			while (!SeedPointDeque.empty())
			{
				int SeedPointIndex = SeedPointDeque.back();
				PointT SeedPoint = m_PointCloud->points[SeedPointIndex];
				Cluster.push_back(SeedPointIndex);
				SeedPointDeque.pop_back();
				UnClusteredPointSet.erase(SeedPointIndex);
				std::vector<int> NearestPointIndexVector(K);
				std::vector<float> NearestPointDistanceVector(K);
				KDTree.nearestKSearch(SeedPoint, K, NearestPointIndexVector, NearestPointDistanceVector);
				for (int k = 0; k < NearestPointIndexVector.size(); k++)
				{
					int NearestPointIndex = NearestPointIndexVector[k];
					PointT NearestPoint = m_PointCloud->points[NearestPointIndex];
					if (UnClusteredPointSet.find(NearestPointIndex) != UnClusteredPointSet.end() && NearestPoint.a == SeedPoint.a)
					{
						UnClusteredPointSet.erase(NearestPointIndex);
						SeedPointDeque.emplace_back(NearestPointIndex);
					}
				}
			}
			Clusters.push_back(Cluster);
		}
	}

	return Clusters;
}

std::vector<std::set<int>> CEvaluateSegmentation::_EvaluateClusterLabel(std::vector<std::set<int>>& vCluster)
{
	return std::vector<std::set<int>>();
}

int CEvaluateSegmentation::_findMaxCountLabelInNeighbours(std::set<int>& vCluster)
{
	return 0;
}
