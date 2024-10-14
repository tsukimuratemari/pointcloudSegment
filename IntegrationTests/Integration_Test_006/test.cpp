#include "pch.h"
#include "Common.h"
#include "Utils.h"
#include "experimenter.h"
#include "online_rf.h"
#include "ProjectionElevation.h"
#include "ElevationDifference.h"
#include "ColorEntropy.h"
#include "DoN.h"
#include "LAB.h"
#include "Covariance.h"
#include "ZDirectionDensity.h"

std::string TESTDATADIR = "E:\\desktop\\Paper\\Exp\\resist_noise_level\\binaryclass\\0\\";
std::string InitialSegmentationResultPath = TESTDATADIR + "Scene.txt";
std::string FeaturePath = TESTDATADIR + "Features\\AllFeatures.txt";
std::string VehicleFeaturePath = TESTDATADIR + "Features\\VehicleFeatures.txt";
std::string GroundFeaturePath = TESTDATADIR + "Features\\GroundFeatures.txt";
std::string BuildingFeaturePath = TESTDATADIR + "Features\\BuildingFeatures.txt";
std::string TreeFeaturePath = TESTDATADIR + "Features\\TreeFeatures.txt";

//TEST(Scene1, fit_initial_segmentation_result)
//{
//	std::string ConfFilePath = "..\\..\\OnlineLearning\\conf\\omcb.conf";
//	Hyperparameters HP(ConfFilePath);
//	
//	std::vector<Result> Results;
//	ESemanticCategory SemanticLabel;
//	CPointCloudTool PointCloudTool;
//	int LabelIndex, NumFeatures, FirstFeatureIndex;
//	std::vector<vector<double>> InitialSegmentationResult;
//
//	/************* fit ground  ***************/
//	SemanticLabel = ESemanticCategory::Ground;
//	PointCloudTool.loadOriginArray(GroundFeaturePath, InitialSegmentationResult);
//	auto InitialTrainData4GroundBinary = InitialSegmentationResult;
//	LabelIndex = 10;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4GroundBinary);
//	DataSet GroundDataSet;
//	NumFeatures = 4;
//	GroundDataSet.setNumFeatures(NumFeatures);
//	FirstFeatureIndex = 6;
//	GroundDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
//	GroundDataSet.setLaeblIndex(LabelIndex);
//	GroundDataSet.load(InitialTrainData4GroundBinary);
//	Classifier* GroundModel = new OnlineRF(HP, GroundDataSet.m_numClasses, GroundDataSet.m_numFeatures, GroundDataSet.m_minFeatRange, GroundDataSet.m_maxFeatRange);
//
//	train(GroundModel, GroundDataSet, HP);
//	Results = test(GroundModel, GroundDataSet, HP);
//	PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4GroundBinary, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "FittingResult\\"+"GroundApproaching.txt", InitialTrainData4GroundBinary);
//	delete GroundModel;
//
//	/************* fit tree  ***************/
//	SemanticLabel = ESemanticCategory::Tree;
//	PointCloudTool.loadOriginArray(TreeFeaturePath, InitialSegmentationResult);
//	auto InitialTrainData4TreeBinary = InitialSegmentationResult;
//	LabelIndex = 11;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4TreeBinary);
//	DataSet TreeDataSet;
//	NumFeatures = 5;
//	TreeDataSet.setNumFeatures(NumFeatures);
//	FirstFeatureIndex = 6;
//	TreeDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
//	TreeDataSet.setLaeblIndex(LabelIndex);
//	TreeDataSet.load(InitialTrainData4TreeBinary);
//	Classifier* TreeModel = new OnlineRF(HP, TreeDataSet.m_numClasses, TreeDataSet.m_numFeatures, TreeDataSet.m_minFeatRange, TreeDataSet.m_maxFeatRange);
//
//	train(TreeModel, TreeDataSet, HP);
//	Results = test(TreeModel, TreeDataSet, HP);
//	PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4TreeBinary, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "FittingResult\\" + "TreeApproaching.txt", InitialTrainData4TreeBinary);
//	delete TreeModel;
//
//	/************* fit building  ***************/
//	SemanticLabel = ESemanticCategory::Building;
//	PointCloudTool.loadOriginArray(BuildingFeaturePath, InitialSegmentationResult);
//	auto InitialTrainData4BuildingBinary = InitialSegmentationResult;
//	LabelIndex = 10;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4BuildingBinary);
//	DataSet BuildingDataSet;
//	NumFeatures = 4;
//	BuildingDataSet.setNumFeatures(NumFeatures);
//	FirstFeatureIndex = 6;
//	BuildingDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
//	BuildingDataSet.setLaeblIndex(LabelIndex);
//	BuildingDataSet.load(InitialTrainData4BuildingBinary);
//	Classifier* BuildingModel = new OnlineRF(HP, BuildingDataSet.m_numClasses, BuildingDataSet.m_numFeatures, BuildingDataSet.m_minFeatRange, BuildingDataSet.m_maxFeatRange);
//
//	train(BuildingModel, BuildingDataSet, HP);
//	Results = test(BuildingModel, BuildingDataSet, HP);
//	PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4BuildingBinary, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "FittingResult\\" + "BuildingApproaching.txt", InitialTrainData4BuildingBinary);
//	delete BuildingModel;
//
//	/************* fit vehicle  ***************/
//	SemanticLabel = ESemanticCategory::Vehicle;
//	PointCloudTool.loadOriginArray(VehicleFeaturePath, InitialSegmentationResult);
//	auto InitialTrainData4VehicleBinary = InitialSegmentationResult;
//	LabelIndex = 9;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4VehicleBinary);
//	DataSet VehicleDataSet;
//	NumFeatures = 3;
//	VehicleDataSet.setNumFeatures(NumFeatures);
//	FirstFeatureIndex = 6;
//	VehicleDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
//	VehicleDataSet.setLaeblIndex(LabelIndex);
//	VehicleDataSet.load(InitialTrainData4VehicleBinary);
//	Classifier* VehicleModel = new OnlineRF(HP, VehicleDataSet.m_numClasses, VehicleDataSet.m_numFeatures, VehicleDataSet.m_minFeatRange, VehicleDataSet.m_maxFeatRange);
//
//	train(VehicleModel, VehicleDataSet, HP);
//	Results = test(VehicleModel, VehicleDataSet, HP);
//	PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4VehicleBinary, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "FittingResult\\" + "VehicleApproaching.txt", InitialTrainData4VehicleBinary);
//	delete VehicleModel;
//}

//TEST(Scene1, merge_binary_fitting_result)
//{
//	std::string FittingResultsFloder = TESTDATADIR + "FittingResult\\";
//	std::vector<std::string> Filenames = { "GroundApproaching.txt", "TreeApproaching.txt", "BuildingApproaching.txt", "VehicleApproaching.txt" };
//
//	auto readFloatData = [](const std::string& vFilename)
//	{
//		std::vector<std::vector<float>> Data;
//
//		std::ifstream File(vFilename);
//		if (!File.is_open())
//		{
//			std::cout << "Failed to open file: " << vFilename << std::endl;
//			return Data;
//		}
//
//		std::string Line;
//		while (std::getline(File, Line))
//		{
//			std::vector<float> Row;
//			std::istringstream Iss(Line);
//			float Value;
//
//			while (Iss >> Value)
//			{
//				Row.push_back(Value);
//			}
//
//			Data.push_back(Row);
//		}
//
//		File.close();
//
//		return Data;
//	};
//
//	auto mapBinaryLabel2MultiLabels = [&](const std::string& vBinaryResultPath, const int vMultiLabelValue)
//	{
//		std::vector<std::vector<float>> PointCloud=readFloatData(vBinaryResultPath);
//		if (PointCloud.empty()||PointCloud[0].empty())
//		{
//			return PointCloud;
//		}
//
//		int RowLength = PointCloud[0].size();
//		for (int i = 0;i < PointCloud.size(); i++)
//		{
//			if (int(PointCloud[i][RowLength - 2]) == 1)
//			{
//				PointCloud[i][6] = vMultiLabelValue;
//			}
//			else
//			{
//				PointCloud[i][6] = -1;
//			}
//			PointCloud[i].resize(7);
//		}
//	};
//
//	auto Ground = mapBinaryLabel2MultiLabels(FittingResultsFloder+ Filenames[0], 0);
//	auto Tree = mapBinaryLabel2MultiLabels(FittingResultsFloder + Filenames[1], 2);
//	auto Building = mapBinaryLabel2MultiLabels(FittingResultsFloder + Filenames[2], 3);
//	auto Vehicle = mapBinaryLabel2MultiLabels(FittingResultsFloder + Filenames[3], 4);
//
//	if (!(Ground.size() == Tree.size() && Ground.size() == Building.size() && Ground.size() == Vehicle.size()))
//	{
//		std::cout << "The size of binary results are not same" << std::endl;
//	}
//
//	auto mergeBinaryResult = [&]()
//	{
//		for (int i = 0; i < Ground.size(); i++)
//		{
//			float MergedLabel = std::max(Ground[i][6], std::max(Tree[i][6], std::max(Building[i][6], Vehicle[i][6])));
//			Ground[i][6] = MergedLabel;
//		}
//	};
//
//	mergeBinaryResult();
//
//	auto writeFloatData = [](const std::string& vFilename, const std::vector<std::vector<float>>& vData)
//	{
//		std::ofstream File(vFilename);
//		if (!File.is_open())
//		{
//			std::cout << "Failed to open file: " << vFilename << std::endl;
//			return;
//		}
//
//		for (const auto& Row : vData)
//		{
//			for (std::size_t i = 0; i < Row.size(); ++i)
//			{
//				File << Row[i];
//				if (i != Row.size() - 1)
//				{
//					File << " ";
//				}
//			}
//			File << std::endl;
//		}
//		File.close();
//	};
//
//	std::string SavePath = FittingResultsFloder + "mergedFittngResult.txt";
//	writeFloatData(SavePath, Ground);
//}

TEST(Scene1, compute_fitting_Acc)
{
	std::string MergedFittingResultPath = TESTDATADIR + "FittingResult\\" + "mergedFittngResult.txt";
	auto readFloatData = [](const std::string& vFilename)
	{
		std::vector<std::vector<float>> Data;

		std::ifstream File(vFilename);
		if (!File.is_open())
		{
			std::cout << "Failed to open file: " << vFilename << std::endl;
			return Data;
		}

		std::string Line;
		while (std::getline(File, Line))
		{
			std::vector<float> Row;
			std::istringstream Iss(Line);
			float Value;

			while (Iss >> Value)
			{
				Row.push_back(Value);
			}

			Data.push_back(Row);
		}

		File.close();

		return Data;
	};

	auto MergeFittingResult = readFloatData(MergedFittingResultPath);
	auto InitialSegmentationResult= readFloatData(InitialSegmentationResultPath);

	auto computeAcc = [&]()
	{
		if (MergeFittingResult.size() != InitialSegmentationResult.size())
		{
			std::cout << "The MergeFittingResult's size is not the smae of InitialSegmentationResult!" << std::endl;
		}

		int FittingCount = 0;
		for (int i = 0; i < MergedFittingResultPath.size(); i++)
		{
			if (int(MergeFittingResult[i][6]) == int(InitialSegmentationResult[i][6]))
			{
				FittingCount++;
			}
		}

		return (float)FittingCount / MergedFittingResultPath.size();
	};

	std::cout << "FittingAcc:" << computeAcc() << std::endl;
}

TEST(Exp_BinaryRF_vs_MultiRF, compute_Acc_by_map)
{
	auto __loadPredictedCloud = [](const std::string& vCloudTXTFilePath, PointCloudT::Ptr& voCloud)
	{
		std::ifstream Reader(vCloudTXTFilePath);
		std::string Line;
		while (std::getline(Reader, Line))
		{
			std::stringstream SS(Line);
			std::string TMP;
			PointT Point;
			std::vector<double> String2Double;
			while (std::getline(SS, TMP, ' '))
			{
				String2Double.push_back(std::stod(TMP));
			}

			Point.x = String2Double[0];
			Point.y = String2Double[1];
			Point.z = String2Double[2];
			Point.r = static_cast<int>(String2Double[3]);
			Point.g = static_cast<int>(String2Double[4]);
			Point.b = static_cast<int>(String2Double[5]);
			Point.a = static_cast<int>(String2Double[String2Double.size() - 1]);
			voCloud->push_back(Point);
		}
		Reader.close();
	};

	auto __loadGroundTruthCloud = [](const std::string& vCloudTXTFilePath, PointCloudT::Ptr& voCloud)
	{
		std::ifstream Reader(vCloudTXTFilePath);
		std::string Line;
		while (std::getline(Reader, Line))
		{
			std::stringstream SS(Line);
			std::string TMP;
			PointT Point;
			std::vector<double> String2Double;
			while (std::getline(SS, TMP, ' '))
			{
				String2Double.push_back(std::stod(TMP));
			}

			Point.x = String2Double[0];
			Point.y = String2Double[1];
			Point.z = String2Double[2];
			Point.r = static_cast<int>(String2Double[3]);
			Point.g = static_cast<int>(String2Double[4]);
			Point.b = static_cast<int>(String2Double[5]);
			Point.a = static_cast<int>(String2Double[String2Double.size() - 1]);
			voCloud->push_back(Point);
		}
		Reader.close();
	};

	std::string PredictedCloudPath = TESTDATADIR + "FittingResult\\" + "mergedFittngResult.txt";
	PointCloudT::Ptr pPredictedCloud = std::make_shared<PointCloudT>();
	__loadPredictedCloud(PredictedCloudPath, pPredictedCloud);

	std::string GroundTruthCloudPath = "E:\\desktop\\Paper\\Exp\\binaryRF_vs_multiRF\\multiRF\\Scene2\\GT.txt";
	PointCloudT::Ptr pGroundTruthCloud = std::make_shared<PointCloudT>();
	__loadGroundTruthCloud(GroundTruthCloudPath, pGroundTruthCloud);

	computeAccByMapPredictedCloud2Groundtruth(pPredictedCloud, pGroundTruthCloud);
}

TEST(Exp_BinaryRF_vs_MultiRF, map_labels_between_pointclouds)
{
	auto __loadPredictedCloud = [](const std::string& vCloudTXTFilePath, PointCloudT::Ptr& voCloud)
	{
		std::ifstream Reader(vCloudTXTFilePath);
		std::string Line;
		while (std::getline(Reader, Line))
		{
			std::stringstream SS(Line);
			std::string TMP;
			PointT Point;
			std::vector<double> String2Double;
			while (std::getline(SS, TMP, ' '))
			{
				String2Double.push_back(std::stod(TMP));
			}

			Point.x = String2Double[0];
			Point.y = String2Double[1];
			Point.z = String2Double[2];
			Point.r = static_cast<int>(String2Double[3]);
			Point.g = static_cast<int>(String2Double[4]);
			Point.b = static_cast<int>(String2Double[5]);
			Point.a = static_cast<int>(String2Double[String2Double.size() - 1]);
			voCloud->push_back(Point);
		}
		Reader.close();
	};

	auto __loadGroundTruthCloud = [](const std::string& vCloudTXTFilePath, PointCloudT::Ptr& voCloud)
	{
		std::ifstream Reader(vCloudTXTFilePath);
		std::string Line;
		while (std::getline(Reader, Line))
		{
			std::stringstream SS(Line);
			std::string TMP;
			PointT Point;
			std::vector<double> String2Double;
			while (std::getline(SS, TMP, ' '))
			{
				String2Double.push_back(std::stod(TMP));
			}

			Point.x = String2Double[0];
			Point.y = String2Double[1];
			Point.z = String2Double[2];
			Point.r = static_cast<int>(String2Double[3]);
			Point.g = static_cast<int>(String2Double[4]);
			Point.b = static_cast<int>(String2Double[5]);
			Point.a = static_cast<int>(String2Double[String2Double.size() - 1]);
			voCloud->push_back(Point);
		}
		Reader.close();
	};

	std::string PredictedCloudPath = "E:\\desktop\\Paper\\Exp\\binaryRF_vs_multiRF\\binaryRF\\Scene3\\Interactions\\ResultAfterPostProcessing.txt";
	PointCloudT::Ptr pPredictedCloud = std::make_shared<PointCloudT>();
	__loadPredictedCloud(PredictedCloudPath, pPredictedCloud);

	std::string GroundTruthCloudPath = "E:\\desktop\\Paper\\Exp\\binaryRF_vs_multiRF\\binaryRF\\Scene3\\SubSample_mapd_Label_Randlanet_block_0_2.txt";
	PointCloudT::Ptr pGroundTruthCloud = std::make_shared<PointCloudT>();
	__loadGroundTruthCloud(GroundTruthCloudPath, pGroundTruthCloud);

	mapLabelBetweenPointClouds(pPredictedCloud, pGroundTruthCloud);
}