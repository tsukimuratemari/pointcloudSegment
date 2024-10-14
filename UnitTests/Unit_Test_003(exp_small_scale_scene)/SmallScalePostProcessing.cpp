#include "pch.h"
#include "SmallScalePostProcessing.h"


CSmallScalePostProcessing::CSmallScalePostProcessing(std::string vFilePath, int vLabelIndex)
{
	pPointCloud = std::make_shared<PCLPointCloudType>();
	loadPointCloud(vFilePath, vLabelIndex);
	
}

CSmallScalePostProcessing::~CSmallScalePostProcessing()
{

}

void CSmallScalePostProcessing::mixDLAndManualConfidenceResult(std::vector<std::vector<double>>& viClusterResult)
{

}

void CSmallScalePostProcessing::loadPointCloud(std::string& vFilePath, int vLabelIndex)
{
	std::ifstream Reader(vFilePath);
	std::string Line;
	int Index = 0;
	while (std::getline(Reader, Line))
	{
		std::stringstream SS(Line);
		std::string TMP;
		PCLPointType Point;
		std::vector<double> String2Double;
		while (std::getline(SS, TMP, ' '))
		{
			String2Double.push_back(std::stod(TMP));
		}

		Point.x = String2Double[0];
		Point.y = String2Double[1];
		Point.z = String2Double[2];
		Point.r = static_cast<std::uint32_t>(String2Double[3]);
		Point.g = static_cast<std::uint32_t>(String2Double[4]);
		Point.b = static_cast<std::uint32_t>(String2Double[5]);
		Point.label = static_cast<std::uint32_t>(String2Double[vLabelIndex]);
		m_Confidence.insert(std::pair<int, float>(Index, String2Double[vLabelIndex + 1]));
		pPointCloud->push_back(Point);
		Index++;
	}
	Reader.close();
	std::cout << "load " << vFilePath <<" success." << std::endl;
}

void CSmallScalePostProcessing::computeAcc(PCLPointCloudType::Ptr & viPredicted, std::string& viGTPath)
{
	int GTLabelIndex = 6;
	std::vector<int>GT;
	std::ifstream Reader(viGTPath);
	std::string Line;
	while (std::getline(Reader, Line))
	{
		std::stringstream SS(Line);
		std::string TMP;
		PCLPointType Point;
		std::vector<double> String2Double;
		while (std::getline(SS, TMP, ' '))
		{
			String2Double.push_back(std::stod(TMP));
		}
		GT.push_back(static_cast<int>(String2Double[GTLabelIndex]));
	}
	Reader.close();
	int CorrectNum = 0;
	for (int i = 0; i < GT.size(); i++)
	{
		if ((int)viPredicted->points[i].label == GT[i])
			CorrectNum++;
	}
	std::cout << "Acc:  " << float(CorrectNum) / GT.size() << std::endl;
}

void CSmallScalePostProcessing::savePointCloud(std::string& vFilePath)
{
	std::ofstream Writer(vFilePath);
	for (int i = 0; i < pPointCloud->points.size(); i++)
	{
		PCLPointType Point = pPointCloud->points.at(i);
		Writer << static_cast<std::float_t>(Point.x) << " ";
		Writer << static_cast<std::float_t>(Point.y) << " ";
		Writer << static_cast<std::float_t>(Point.z) << " ";
		Writer << static_cast<std::uint32_t>(Point.r) << " ";
		Writer << static_cast<std::uint32_t>(Point.g) << " ";
		Writer << static_cast<std::uint32_t>(Point.b) << " ";
		Writer << static_cast<std::uint32_t>(Point.label) << "\n";
		//std::cout << static_cast<std::float_t>(Point.x) << std::endl;
	}
	Writer.close();
}
void CSmallScalePostProcessing::savePointCloud(std::string& vFilePath, std::vector<int> viPointCloudIndexVector)
{
	std::ofstream Writer(vFilePath);
	for (int i = 0; i < viPointCloudIndexVector.size(); i++)
	{
		PCLPointType Point = pPointCloud->points.at(viPointCloudIndexVector[i]);
		Writer << static_cast<std::float_t>(Point.x) << " ";
		Writer << static_cast<std::float_t>(Point.y) << " ";
		Writer << static_cast<std::float_t>(Point.z) << " ";
		Writer << static_cast<std::uint32_t>(Point.r) << " ";
		Writer << static_cast<std::uint32_t>(Point.g) << " ";
		Writer << static_cast<std::uint32_t>(Point.b) << " ";
		Writer << static_cast<std::uint32_t>(Point.label) << "\n";
		//std::cout << static_cast<std::float_t>(Point.x) << std::endl;
	}
	Writer.close();
}

void CSmallScalePostProcessing::__preUnclassifiedPoints()
{
	pcl::KdTreeFLANN<PCLPointType> KDTree;
	KDTree.setInputCloud(pPointCloud);
	int K = 300;
	std::set<int> UnClusteredPointSet;
	for (int i = 0; i < pPointCloud->size(); i++)UnClusteredPointSet.emplace(i);
	//search all cluster and save in a vector
	for (int i = 0; i < pPointCloud->size(); i++)
	{
		if (m_Confidence.at(i) < 1e-2)
		{
			std::vector<int> NearestPointIndexVector(K);
			std::vector<float> NearestPointDistanceVector(K);
			if (KDTree.nearestKSearch(pPointCloud->points[i], K, NearestPointIndexVector, NearestPointDistanceVector) > 0)
			{
				//decide small cluster postprocess label
				int GroundCount = 0, TreeCount = 0, BuildingCount = 0, VehicleCount = 0;
				for (int k = 0; k < NearestPointIndexVector.size(); k++)
				{
					if (m_Confidence.at(NearestPointIndexVector[k]) > 1e-2)
					{
						switch (static_cast<ESemanticCategory>(pPointCloud->points[NearestPointIndexVector[k]].label))
						{
						case ESemanticCategory::Ground:GroundCount++; break;
						case ESemanticCategory::Tree:TreeCount++; break;
						case ESemanticCategory::Building:BuildingCount++; break;
						case ESemanticCategory::Vehicle:VehicleCount++; break;
						}
					}
				}
				ESemanticCategory ClusterNewLabel;
				if (GroundCount >= TreeCount && GroundCount >= BuildingCount && GroundCount >= VehicleCount)ClusterNewLabel = ESemanticCategory::Ground;
				else if (TreeCount >= GroundCount && TreeCount >= BuildingCount && TreeCount >= VehicleCount)ClusterNewLabel = ESemanticCategory::Tree;
				else if (BuildingCount >= GroundCount && BuildingCount >= TreeCount && BuildingCount >= VehicleCount)ClusterNewLabel = ESemanticCategory::Building;
				else if (VehicleCount >= GroundCount && VehicleCount >= TreeCount && VehicleCount >= BuildingCount)ClusterNewLabel = ESemanticCategory::Vehicle;
				
				pPointCloud->points[i].label = static_cast<uint32_t>(ClusterNewLabel);
				m_Confidence.at(i) = 0.5;

			}
		}
	}
}

void CSmallScalePostProcessing::clusteringPointCloud(std::vector<std::vector<int>>& voClusterResult)
{
	//__preUnclassifiedPoints();
	pcl::KdTreeFLANN<PCLPointType> KDTree;
	KDTree.setInputCloud(pPointCloud);
	int K = 10;

	std::set<int> UnClusteredPointSet;
	for (int i = 0; i < pPointCloud->size(); i++)UnClusteredPointSet.emplace(i);
	//search all cluster and save in a vector
	for (int i = 0; i < pPointCloud->size(); i++)
	{
		if (UnClusteredPointSet.find(i) != UnClusteredPointSet.end())
		{
			//search a cluster
			std::deque<int> SeedPointDeque;
			std::vector<int> Cluster;
			SeedPointDeque.push_back(i);
			while (!SeedPointDeque.empty())
			{
				int SeedPointIndex= SeedPointDeque.back();
				PCLPointType SeedPoint = pPointCloud->points[SeedPointIndex];
				Cluster.push_back(SeedPointIndex);
				SeedPointDeque.pop_back();
				UnClusteredPointSet.erase(SeedPointIndex);
				std::vector<int> NearestPointIndexVector(K);
				std::vector<float> NearestPointDistanceVector(K);
				KDTree.nearestKSearch(SeedPoint, K, NearestPointIndexVector, NearestPointDistanceVector);
				for (int k = 0; k < NearestPointIndexVector.size(); k++)
				{
					int NearestPointIndex = NearestPointIndexVector[k];
					PCLPointType NearestPoint = pPointCloud->points[NearestPointIndex];
					if (UnClusteredPointSet.find(NearestPointIndex) != UnClusteredPointSet.end() && NearestPoint.label == SeedPoint.label)
					{
						UnClusteredPointSet.erase(NearestPointIndex);
						SeedPointDeque.emplace_back(NearestPointIndex);
					}
				}
			}
			voClusterResult.push_back(Cluster);
		}
	}

	for (int i = 0; i < voClusterResult.size(); i++)
	{
		if (voClusterResult[i].size() < 50)
		{
			std::vector<int> TempCluster = voClusterResult[i];
			//search a bounding box cluster
			std::deque<int> SeedPointDeque;
			std::vector<int> BoundingPoints;
			SeedPointDeque.push_back(TempCluster[0]);
			for (int k = 0; k < TempCluster.size(); k++)
			{
				std::vector<int> NearestPointIndexVector(K);
				std::vector<float> NearestPointDistanceVector(K);
				KDTree.nearestKSearch(pPointCloud->points[TempCluster[k]], K, NearestPointIndexVector, NearestPointDistanceVector) > 0;
				for (int m = 0; m < NearestPointIndexVector.size(); m++)
				{
					if (std::find(TempCluster.begin(), TempCluster.end(), NearestPointIndexVector[m]) == TempCluster.end() && std::find(BoundingPoints.begin(), BoundingPoints.end(), NearestPointIndexVector[m]) == BoundingPoints.end())
						BoundingPoints.push_back(NearestPointIndexVector[m]);
				}
			}
			//decide small cluster postprocess label
			int GroundCount = 0, TreeCount = 0, BuildingCount = 0, VehicleCount = 0;
			for (int k = 0; k < BoundingPoints.size(); k++)
			{
				switch (static_cast<ESemanticCategory>(pPointCloud->points[BoundingPoints[k]].label))
				{
				case ESemanticCategory::Ground:GroundCount++; break;
				case ESemanticCategory::Tree:TreeCount++; break;
				case ESemanticCategory::Building:BuildingCount++; break;
				case ESemanticCategory::Vehicle:VehicleCount++; break;
				}
			}
			ESemanticCategory ClusterNewLabel;
			if (GroundCount >= TreeCount && GroundCount >= BuildingCount && GroundCount >= VehicleCount)ClusterNewLabel = ESemanticCategory::Ground;
			else if (TreeCount >= GroundCount && TreeCount >= BuildingCount && TreeCount >= VehicleCount)ClusterNewLabel = ESemanticCategory::Tree;
			else if (BuildingCount >= GroundCount && BuildingCount >= TreeCount && BuildingCount >= VehicleCount)ClusterNewLabel = ESemanticCategory::Building;
			else if (VehicleCount >= GroundCount && VehicleCount >= TreeCount && VehicleCount >= BuildingCount)ClusterNewLabel = ESemanticCategory::Vehicle;
			for (int k = 0; k < TempCluster.size(); k++)
			{
				pPointCloud->points[TempCluster[k]].label = static_cast<uint32_t>(ClusterNewLabel);
			}
		}
	}



}

void CSmallScalePostProcessing::processSmallScaleCluster(std::vector<std::vector<int>>& viClusterResult)
{

}
