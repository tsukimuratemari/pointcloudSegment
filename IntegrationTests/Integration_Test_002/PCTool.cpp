#include "pch.h"
#include "PCTool.h"



void CPCTool::extractFeatures()
{
	m_pCloud.reset();
	m_pCloud = std::make_shared<PointCloudT>();
	loadCloud(InitialPath, m_pCloud);

	_HIVE_EARLY_EXIT(m_pCloud && m_pCloud->empty(), "point cloud shouldn't be initialized!");

	int Width = 128, Height = 128;
	CProjectionElevation PEr(m_pCloud, Width, Height);
	std::unordered_map<int, float> ProjectElevations;
	PEr.compute(ProjectElevations);

	CElevationDifference EDr(m_pCloud);
	std::unordered_map<int, float> ElevationDifferences;
	EDr.compute(ElevationDifferences);
	std::unordered_map<int, float> ProjectElevationDifferences;
	EDr.computeBasedProjectElevation(ProjectElevationDifferences);

	CColorEntropy CEr(m_pCloud);
	std::unordered_map<int, float> ColorEntroys;
	float Radius = 0.7f;
	CEr.compute(ColorEntroys, Radius);

	float SmallRadius = 0.5f, LargeRadius = 5.0f;
	DoNInputPointCloudType::Ptr DoNInputCloud(new DoNInputPointCloudType);
	pcl::copyPointCloud(*m_pCloud, *DoNInputCloud);
	CDoN DoNr(DoNInputCloud);
	std::unordered_map<int, float> DoNs;
	DoNr.computeDoN(SmallRadius, LargeRadius, DoNs);

	CLAB LABr(m_pCloud);
	std::unordered_map<int, std::array<float, 3>> LABs;
	LABr.compute(LABs);

	CCovariance Covar(m_pCloud);
	std::unordered_map<int, std::vector<float>> CovarianceFeature;
	Radius = 2.0f;
	Covar.computeCovariance(Radius, CovarianceFeature);

	CZDirectionDensity ZDens(m_pCloud);
	std::unordered_map<int, float> ZDensityFeature;
	Radius = 1.0f;
	ZDens.computeZDirectionDensity(Radius, ZDensityFeature);

	std::unordered_map<int, SFeatures> Features;
	Features.reserve(m_pCloud->size());
	for (int i = 0; i < m_pCloud->size(); i++)
	{
		Features.emplace(i, SFeatures(
			ProjectElevations.at(i),
			ElevationDifferences.at(i),
			ProjectElevationDifferences.at(i),
			ColorEntroys.at(i),
			DoNs.at(i),
			LABs.at(i),
			CovarianceFeature.at(i)[5],
			ZDensityFeature.at(i)));
	}

	__writePositionAndFeatures(Features, FeaturePath);
	__writeVehicleFeatures(Features, VehicleFeaturePath);
	__writeGroundFeatures(Features, GroundFeaturePath);
	__writeBuildingFeatures(Features, BuildingFeaturePath);
	__writeTreeFeatures(Features, TreeFeaturePath);
}

void CPCTool::trainORFModel(std::vector<std::pair<std::string, std::vector<ESemanticCategory>>>& viTrainDir, std::vector<std::vector<std::string>>& viTestDir)
{
	Hyperparameters HP(ConfFilePath);
	for (int i = 0; i < viTrainDir.size(); i++)
	{
		std::vector<std::vector<double>> TrainPointCloud;
		int LastCategoryLabelIndex = -1;
		ESemanticCategory LastTestCategory = ESemanticCategory::Undefined;
		setInitialPath(viTrainDir[i].first);

		if (viTestDir[i].size() == 0)continue;

		for (auto it : viTrainDir[i].second)
		{
			std::cout << std::endl;
			std::vector<std::vector<double>> TestPointCloud;
			ESemanticCategory SemanticLabel;
			switch (it)
			{
			case ESemanticCategory::Vehicle:
			{
				SemanticLabel = ESemanticCategory::Vehicle;
				PointCloudTool.loadOriginArray(VehicleFeaturePath, TrainPointCloud);
				PointCloudTool.setLabelColumnIndex(VehicleConfig._LabelIndex);
				PointCloudTool.setConfidenceColumnIndex(VehicleConfig._ConfidenceColumnIndex);
				PointCloudTool.binarizeDLTrainData(SemanticLabel, TrainPointCloud);
				DataSet TrainData(VehicleConfig._NumFeatures, VehicleConfig._FirstFeatureColumnIndex, VehicleConfig._LabelIndex);
				TrainData.load(TrainPointCloud);
				Classifier* Model = new OnlineRF(HP, TrainData.m_numClasses, TrainData.m_numFeatures, TrainData.m_minFeatRange, TrainData.m_maxFeatRange);
				train(Model, TrainData, HP);

				for (int k = 0; k < viTestDir[i].size(); k++)
				{
					setSubsequentDir(viTestDir[i][k]);
					vector<Result> Results;
					
					PointCloudTool.loadOriginArray(SubsequentDir + "VehicleFeatures.txt", TestPointCloud);
					if (LastTestCategory != ESemanticCategory::Undefined)
					{
						std::vector<vector<double>> LastCorrectionResult;
						PointCloudTool.loadOriginArray(ManualResultPath + returnSemanticString(LastTestCategory) + "Result.txt", LastCorrectionResult);
						PointCloudTool.changeLabelAndConf2AnotherDataset(TestPointCloud, LastCorrectionResult, LastCategoryLabelIndex);
					}
					std::vector<std::vector<double>> TempPointCloud = TestPointCloud;
					PointCloudTool.binarizeDLTrainData(SemanticLabel, TempPointCloud);
					
					DataSet TestData(VehicleConfig._NumFeatures, VehicleConfig._FirstFeatureColumnIndex, VehicleConfig._LabelIndex);
					TestData.load(TempPointCloud);
					Results = test(Model, TestData, HP);
					
					PointCloudTool.visualizeORFApproachManualResult(TestPointCloud, SemanticLabel, Results);
					PointCloudTool.saveArrayFile(SubsequentDir + "VehicleResult.txt", TestPointCloud);
				}
				LastTestCategory = SemanticLabel;
				LastCategoryLabelIndex = VehicleConfig._LabelIndex;
				delete Model;
				break;
			}
			case ESemanticCategory::Ground:
			{
				SemanticLabel = ESemanticCategory::Ground;
				PointCloudTool.loadOriginArray(GroundFeaturePath, TrainPointCloud);
				PointCloudTool.setLabelColumnIndex(GroundConfig._LabelIndex);
				PointCloudTool.setConfidenceColumnIndex(GroundConfig._ConfidenceColumnIndex);
				PointCloudTool.binarizeDLTrainData(SemanticLabel, TrainPointCloud);
				DataSet TrainData(GroundConfig._NumFeatures, GroundConfig._FirstFeatureColumnIndex, GroundConfig._LabelIndex);
				TrainData.load(TrainPointCloud);
				Classifier* Model = new OnlineRF(HP, TrainData.m_numClasses, TrainData.m_numFeatures, TrainData.m_minFeatRange, TrainData.m_maxFeatRange);
				train(Model, TrainData, HP);

				for (int k = 0; k < viTestDir[i].size(); k++)
				{
					setSubsequentDir(viTestDir[i][k]);
					vector<Result> Results;

					PointCloudTool.loadOriginArray(SubsequentDir + "GroundFeatures.txt", TestPointCloud);
					if (LastTestCategory != ESemanticCategory::Undefined)
					{
						std::vector<vector<double>> LastCorrectionResult;
						PointCloudTool.loadOriginArray(ManualResultPath + returnSemanticString(LastTestCategory) + "Result.txt", LastCorrectionResult);
						PointCloudTool.changeLabelAndConf2AnotherDataset(TestPointCloud, LastCorrectionResult, LastCategoryLabelIndex);
					}
					std::vector<std::vector<double>> TempPointCloud = TestPointCloud;
					PointCloudTool.binarizeDLTrainData(SemanticLabel, TempPointCloud);

					DataSet TestData(GroundConfig._NumFeatures, GroundConfig._FirstFeatureColumnIndex, GroundConfig._LabelIndex);
					TestData.load(TempPointCloud);
					Results = test(Model, TestData, HP);

					PointCloudTool.visualizeORFApproachManualResult(TestPointCloud, SemanticLabel, Results);
					PointCloudTool.saveArrayFile(SubsequentDir + "GroundResult.txt", TestPointCloud);
				}
				LastTestCategory = SemanticLabel;
				LastCategoryLabelIndex = GroundConfig._LabelIndex;
				delete Model;
				break;
			}
			case ESemanticCategory::Tree:
			{
				SemanticLabel = ESemanticCategory::Tree;
				PointCloudTool.loadOriginArray(TreeFeaturePath, TrainPointCloud);
				PointCloudTool.setLabelColumnIndex(TreeConfig._LabelIndex);
				PointCloudTool.setConfidenceColumnIndex(TreeConfig._ConfidenceColumnIndex);
				PointCloudTool.binarizeDLTrainData(SemanticLabel, TrainPointCloud);
				DataSet TrainData(TreeConfig._NumFeatures, TreeConfig._FirstFeatureColumnIndex, TreeConfig._LabelIndex);
				TrainData.load(TrainPointCloud);
				Classifier* Model = new OnlineRF(HP, TrainData.m_numClasses, TrainData.m_numFeatures, TrainData.m_minFeatRange, TrainData.m_maxFeatRange);
				train(Model, TrainData, HP);

				for (int k = 0; k < viTestDir[i].size(); k++)
				{
					setSubsequentDir(viTestDir[i][k]);
					vector<Result> Results;

					PointCloudTool.loadOriginArray(SubsequentDir + "TreeFeatures.txt", TestPointCloud);
					if (LastTestCategory != ESemanticCategory::Undefined)
					{
						std::vector<vector<double>> LastCorrectionResult;
						PointCloudTool.loadOriginArray(ManualResultPath + returnSemanticString(LastTestCategory) + "Result.txt", LastCorrectionResult);
						PointCloudTool.changeLabelAndConf2AnotherDataset(TestPointCloud, LastCorrectionResult, LastCategoryLabelIndex);
					}
					std::vector<std::vector<double>> TempPointCloud = TestPointCloud;
					PointCloudTool.binarizeDLTrainData(SemanticLabel, TempPointCloud);

					DataSet TestData(TreeConfig._NumFeatures, TreeConfig._FirstFeatureColumnIndex, TreeConfig._LabelIndex);
					TestData.load(TempPointCloud);
					Results = test(Model, TestData, HP);

					PointCloudTool.visualizeORFApproachManualResult(TestPointCloud, SemanticLabel, Results);
					PointCloudTool.saveArrayFile(SubsequentDir + "TreeResult.txt", TestPointCloud);
				}
				LastTestCategory = SemanticLabel;
				LastCategoryLabelIndex = TreeConfig._LabelIndex;
				delete Model;
				break;
			}
			case ESemanticCategory::Building:
			{
				SemanticLabel = ESemanticCategory::Building;
				PointCloudTool.loadOriginArray(BuildingFeaturePath, TrainPointCloud);
				PointCloudTool.setLabelColumnIndex(BuildingConfig._LabelIndex);
				PointCloudTool.setConfidenceColumnIndex(BuildingConfig._ConfidenceColumnIndex);
				PointCloudTool.binarizeDLTrainData(SemanticLabel, TrainPointCloud);
				DataSet TrainData(BuildingConfig._NumFeatures, BuildingConfig._FirstFeatureColumnIndex, BuildingConfig._LabelIndex);
				TrainData.load(TrainPointCloud);
				Classifier* Model = new OnlineRF(HP, TrainData.m_numClasses, TrainData.m_numFeatures, TrainData.m_minFeatRange, TrainData.m_maxFeatRange);
				train(Model, TrainData, HP);

				for (int k = 0; k < viTestDir[i].size(); k++)
				{
					setSubsequentDir(viTestDir[i][k]);
					vector<Result> Results;

					PointCloudTool.loadOriginArray(SubsequentDir + "BuildingFeatures.txt", TestPointCloud);
					if (LastTestCategory != ESemanticCategory::Undefined)
					{
						std::vector<vector<double>> LastCorrectionResult;
						PointCloudTool.loadOriginArray(ManualResultPath + returnSemanticString(LastTestCategory) + "Result.txt", LastCorrectionResult);
						PointCloudTool.changeLabelAndConf2AnotherDataset(TestPointCloud, LastCorrectionResult, LastCategoryLabelIndex);
					}
					std::vector<std::vector<double>> TempPointCloud = TestPointCloud;
					PointCloudTool.binarizeDLTrainData(SemanticLabel, TempPointCloud);

					DataSet TestData(BuildingConfig._NumFeatures, BuildingConfig._FirstFeatureColumnIndex, BuildingConfig._LabelIndex);
					TestData.load(TempPointCloud);
					Results = test(Model, TestData, HP);

					PointCloudTool.visualizeORFApproachManualResult(TestPointCloud, SemanticLabel, Results);
					PointCloudTool.saveArrayFile(SubsequentDir + "BuildingResult.txt", TestPointCloud);
				}
				LastTestCategory = SemanticLabel;
				LastCategoryLabelIndex = BuildingConfig._LabelIndex;
				delete Model;
				break;
			}

			}
		}
		std::cout << std::endl << std::endl;;
	}
};

void CPCTool::setInitialPath(std::string& viInitialPath)
{
	InitialPath = viInitialPath;
	TESTDATADIR = viInitialPath.substr(0, viInitialPath.find_last_of('\\') + 1);
	FeaturePath = TESTDATADIR + "AllFeatures.txt";
	VehicleFeaturePath = TESTDATADIR + "VehicleFeatures.txt";
	GroundFeaturePath = TESTDATADIR + "GroundFeatures.txt";
	BuildingFeaturePath = TESTDATADIR + "BuildingFeatures.txt";
	TreeFeaturePath = TESTDATADIR + "TreeFeatures.txt";
}

void CPCTool::setSubsequentDir(std::string& viSubsequentDir)
{
	SubsequentDir = viSubsequentDir;
	ManualResultPath = viSubsequentDir;
}

std::string CPCTool::returnSemanticString(ESemanticCategory viSemantic)
{
	switch (viSemantic)
	{
	case ESemanticCategory::Ground:
		return "Ground";
		break;
	case ESemanticCategory::Vegetation:
		return "Vegetation";
		break;
	case ESemanticCategory::Tree:
		return "Tree";
		break;
	case ESemanticCategory::Building:
		return "Building";
		break;
	case ESemanticCategory::Vehicle:
		return "Vehicle";
		break;
	case ESemanticCategory::River:
		return "River";
		break;
	case ESemanticCategory::Undefined:
		return "Undefined";
		break;
	default:
		break;
	}
}

void CPCTool::__writeTreeFeatures(const std::unordered_map<int, SFeatures>& vFeatures, const std::string& vPath)
{
	std::ofstream Writer(vPath);
	Writer.setf(ios::fixed, ios::floatfield);
	Writer.precision(3);
	for (int i = 0; i < m_pCloud->size(); i++)
	{
		Writer << (double)m_pCloud->points.at(i).x << " ";
		Writer << (double)m_pCloud->points.at(i).y << " ";
		Writer << (double)m_pCloud->points.at(i).z << " ";
		Writer << (int)m_pCloud->points.at(i).r << " ";
		Writer << (int)m_pCloud->points.at(i).g << " ";
		Writer << (int)m_pCloud->points.at(i).b << " ";
		Writer << vFeatures.at(i)._Planarity << " ";
		Writer << vFeatures.at(i)._ZDensity << " ";
		Writer << vFeatures.at(i)._LAB[0] << " ";
		Writer << (int)m_pCloud->points.at(i).a << " ";
		Writer << "\n";
	}
	Writer.close();
}

void CPCTool::__writeBuildingFeatures(const std::unordered_map<int, SFeatures>& vFeatures, const std::string& vPath)
{
	std::ofstream Writer(vPath);
	Writer.setf(ios::fixed, ios::floatfield);
	Writer.precision(3);
	for (int i = 0; i < m_pCloud->size(); i++)
	{
		Writer << (double)m_pCloud->points.at(i).x << " ";
		Writer << (double)m_pCloud->points.at(i).y << " ";
		Writer << (double)m_pCloud->points.at(i).z << " ";
		Writer << (int)m_pCloud->points.at(i).r << " ";
		Writer << (int)m_pCloud->points.at(i).g << " ";
		Writer << (int)m_pCloud->points.at(i).b << " ";
		Writer << vFeatures.at(i)._Planarity << " ";
		Writer << vFeatures.at(i)._ZDensity << " ";
		//Writer << vFeatures.at(i)._LAB[0] << " ";
		Writer << (int)m_pCloud->points.at(i).a << " ";
		Writer << "\n";
	}
	Writer.close();
}

void CPCTool::__writeVehicleFeatures(const std::unordered_map<int, SFeatures>& vFeatures, const std::string& vPath)
{
	std::ofstream Writer(vPath);
	Writer.setf(ios::fixed, ios::floatfield);
	Writer.precision(3);
	for (int i = 0; i < m_pCloud->size(); i++)
	{
		Writer << (double)m_pCloud->points.at(i).x << " ";
		Writer << (double)m_pCloud->points.at(i).y << " ";
		Writer << (double)m_pCloud->points.at(i).z << " ";
		Writer << (int)m_pCloud->points.at(i).r << " ";
		Writer << (int)m_pCloud->points.at(i).g << " ";
		Writer << (int)m_pCloud->points.at(i).b << " ";
		Writer << vFeatures.at(i)._ProjectElevation << " ";
		Writer << vFeatures.at(i)._ProjectElevationDifference << " ";
		Writer << vFeatures.at(i)._DoN << " ";
		Writer << vFeatures.at(i)._ColorEntroy << " ";
		Writer << (int)m_pCloud->points.at(i).a << " ";
		Writer << "\n";
	}
	Writer.close();

}

void CPCTool::__writeGroundFeatures(const std::unordered_map<int, SFeatures>& vFeatures, const std::string& vPath)
{
	std::ofstream Writer(vPath);
	Writer.setf(ios::fixed, ios::floatfield);
	Writer.precision(3);
	for (int i = 0; i < m_pCloud->size(); i++)
	{
		Writer << (double)m_pCloud->points.at(i).x << " ";
		Writer << (double)m_pCloud->points.at(i).y << " ";
		Writer << (double)m_pCloud->points.at(i).z << " ";
		Writer << (int)m_pCloud->points.at(i).r << " ";
		Writer << (int)m_pCloud->points.at(i).g << " ";
		Writer << (int)m_pCloud->points.at(i).b << " ";
		Writer << vFeatures.at(i)._ProjectElevation << " ";
		Writer << vFeatures.at(i)._ProjectElevationDifference << " ";
		Writer << vFeatures.at(i)._DoN << " ";
		Writer << (int)m_pCloud->points.at(i).a << " ";
		Writer << "\n";
	}
	Writer.close();

}

void CPCTool::__writePositionAndFeatures(const std::unordered_map<int, SFeatures>& vFeatures, const std::string& vPath)
{
	std::ofstream Writer(vPath);
	Writer.setf(ios::fixed, ios::floatfield);
	Writer.precision(3);
	for (int i = 0; i < m_pCloud->size(); i++)
	{
		Writer << (double)m_pCloud->points.at(i).x << " ";
		Writer << (double)m_pCloud->points.at(i).y << " ";
		Writer << (double)m_pCloud->points.at(i).z << " ";
		Writer << (int)m_pCloud->points.at(i).r << " ";
		Writer << (int)m_pCloud->points.at(i).g << " ";
		Writer << (int)m_pCloud->points.at(i).b << " ";
		Writer << vFeatures.at(i)._ProjectElevation << " ";
		Writer << vFeatures.at(i)._ElevationDifference << " ";
		Writer << vFeatures.at(i)._ProjectElevationDifference << " ";
		Writer << vFeatures.at(i)._DoN << " ";
		Writer << vFeatures.at(i)._ColorEntroy << " ";
		Writer << vFeatures.at(i)._LAB[0] << " ";
		Writer << vFeatures.at(i)._LAB[1] << " ";
		Writer << vFeatures.at(i)._LAB[2] << " ";
		Writer << vFeatures.at(i)._Planarity << " ";
		Writer << vFeatures.at(i)._ZDensity << " ";
		Writer << "\n";
	}
	Writer.close();
}



CSmallScalePostProcessing::CSmallScalePostProcessing(std::string vFilePath)
{
	pPointCloud = std::make_shared<PCLPointCloudType>();
	loadPointCloud(vFilePath);

}

CSmallScalePostProcessing::~CSmallScalePostProcessing()
{

}

void CSmallScalePostProcessing::mixDLAndManualConfidenceResult(std::vector<std::vector<double>>& viClusterResult)
{

}

void CSmallScalePostProcessing::loadPointCloud(std::string& vFilePath)
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
		Point.label = static_cast<std::uint32_t>(String2Double[String2Double.size() - 2]);
		m_Confidence.insert(std::pair<int, float>(Index, String2Double[String2Double.size() - 1]));
		pPointCloud->push_back(Point);
		Index++;
	}
	Reader.close();
	//std::cout << "load " << vFilePath << " success." << std::endl;
}

void CSmallScalePostProcessing::computeAcc(PCLPointCloudType::Ptr& viPredicted, std::string& viGTPath)
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
	//std::cout << "Acc:  " << float(CorrectNum) / GT.size() << std::endl;
	std::cout << "  " << float(CorrectNum) / GT.size() << "  ";
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
	__preUnclassifiedPoints();
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
				int SeedPointIndex = SeedPointDeque.back();
				PCLPointType SeedPoint = pPointCloud->points[SeedPointIndex];
				Cluster.push_back(SeedPointIndex);
				SeedPointDeque.pop_back();
				UnClusteredPointSet.erase(SeedPointIndex);
				std::vector<int> NearestPointIndexVector(K);
				std::vector<float> NearestPointDistanceVector(K);
				KDTree.nearestKSearch(SeedPoint, K, NearestPointIndexVector, NearestPointDistanceVector) > 0;
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
