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

std::string TESTDATADIR = "C:\\Users\\HPJ\\Desktop\\Paper\\Exp\\Scene1\\subsample_cloud\\Randlanet\\";
std::string InitialSegmentationResultPath = TESTDATADIR + "SubSample_mapd_Label_Randlanet_block_0_1.txt";
std::string FeaturePath = TESTDATADIR + "Features\\AllFeatures.txt";
std::string VehicleFeaturePath = TESTDATADIR + "Features\\VehicleFeatures.txt";
std::string GroundFeaturePath = TESTDATADIR + "Features\\GroundFeatures.txt";
std::string BuildingFeaturePath = TESTDATADIR + "Features\\BuildingFeatures.txt";
std::string TreeFeaturePath = TESTDATADIR + "Features\\TreeFeatures.txt";

TEST(Scene1, correct_randlanet)
{
	std::string InitialSegmentationResultPath = TESTDATADIR + "SubSample_mapd_Label_Randlanet_block_0_1.txt";
	std::string ConfFilePath = "..\\..\\OnlineLearning\\conf\\omcb.conf";
	Hyperparameters HP(ConfFilePath);
	CPointCloudTool PointCloudTool;
	std::vector<vector<double>> InitialSegmentationResult;

	///************* Ground Correction ***************/
	//ESemanticCategory SemanticLabel = ESemanticCategory::Ground;
	//PointCloudTool.loadOriginArray(GroundFeaturePath, InitialSegmentationResult);
	//auto InitialTrainData4GroundBinary = InitialSegmentationResult;
	//int LabelIndex = 10;
	//PointCloudTool.setLabelColumnIndex(LabelIndex);
	//PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4GroundBinary);
	//DataSet GroundDataSet;
	//int NumFeatures = 4, FirstFeatureIndex = 6;
	//GroundDataSet.setNumFeatures(NumFeatures);
	//GroundDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
	//GroundDataSet.setLaeblIndex(LabelIndex);
	//GroundDataSet.load(InitialTrainData4GroundBinary);
	//Classifier* Model = new OnlineRF(HP, GroundDataSet.m_numClasses, GroundDataSet.m_numFeatures, GroundDataSet.m_minFeatRange, GroundDataSet.m_maxFeatRange);

	//train(Model, GroundDataSet, HP);
	//vector<Result> Results = test(Model, GroundDataSet, HP);
	//PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4GroundBinary, Results);
	//PointCloudTool.saveArrayFile(TESTDATADIR + "GroundApproaching.txt", InitialTrainData4GroundBinary);

	////ManualResult approaching stage 1
	//vector<string> PosManualResultFilePathVector;
	//vector<string> NegManualResultFilePathVector;
	//PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\1\\Positive.txt");
	//NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\1\\Negative.txt");

	//std::vector<vector<double>> ManualTrainData;
	//PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
	//DataSet GroundCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
	//GroundCorrectionOne.load(ManualTrainData);
	//train(Model, GroundCorrectionOne, HP);
	//Results.clear();
	//Results = test(Model, GroundDataSet, HP);

	//int ConfidenceColumnIndex = 11;
	//PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
	//PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
	//PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\1\\Result.txt", InitialSegmentationResult);

	////ManualResult approaching stage 2
	//PosManualResultFilePathVector;
	//NegManualResultFilePathVector;
	//PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\2\\Positive.txt");
	//NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\2\\Negative.txt");

	//PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
	//DataSet GroundCorrectionTwo(NumFeatures, FirstFeatureIndex, LabelIndex);
	//GroundCorrectionTwo.load(ManualTrainData);
	//train(Model, GroundCorrectionTwo, HP);
	//Results.clear();
	//Results = test(Model, GroundDataSet, HP);

	//PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
	//PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
	//PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\2\\Result.txt", InitialSegmentationResult);

	//delete Model;

	/************* Building Correction ***************/
	ESemanticCategory SemanticLabel = ESemanticCategory::Building;
	PointCloudTool.loadOriginArray(BuildingFeaturePath, InitialSegmentationResult);
	auto InitialTrainData4BuildingBinary = InitialSegmentationResult;
	int LabelIndex = 10;
	PointCloudTool.setLabelColumnIndex(LabelIndex);
	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4BuildingBinary);
	DataSet BuildingDataSet;
	int NumFeatures = 4, FirstFeatureIndex = 6;
	BuildingDataSet.setNumFeatures(NumFeatures);
	BuildingDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
	BuildingDataSet.setLaeblIndex(LabelIndex);
	BuildingDataSet.load(InitialTrainData4BuildingBinary);
	Classifier*  Model = new OnlineRF(HP, BuildingDataSet.m_numClasses, BuildingDataSet.m_numFeatures, BuildingDataSet.m_minFeatRange, BuildingDataSet.m_maxFeatRange);

	train(Model, BuildingDataSet, HP);
	vector<Result> Results = test(Model, BuildingDataSet, HP);
	PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4BuildingBinary, Results);
	PointCloudTool.saveArrayFile(TESTDATADIR + "BuildingApproaching.txt", InitialTrainData4BuildingBinary);

	//ManualResult approaching stage 1

}

TEST(Scene1, compute_correctd_Acc)
{
	std::string CorrectedResultPath = TESTDATADIR + "Interactions\\" +"2\\"+"Result.txt";
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

	auto CorrectedResult = readFloatData(CorrectedResultPath);
	auto GroundthResult= readFloatData(InitialSegmentationResultPath);

	auto computeAcc = [&]()
	{
		if (CorrectedResult.size() != GroundthResult.size())
		{
			std::cout << "The CorrectedResult's size is not the smae of InitialSegmentationResult!" << std::endl;
		}

		int FittingCount = 0;
		int CorrectedLabelIndex = CorrectedResult[0].size()-2;
		int GroundthLabelIndex = GroundthResult[0].size() - 1;
		for (int i = 0; i < CorrectedResult.size(); i++)
		{
			if (int(CorrectedResult[i][CorrectedLabelIndex]) == int(GroundthResult[i][GroundthLabelIndex]))
			{
				FittingCount++;
			}
		}

		return (float)FittingCount / GroundthResult.size();
	};

	std::cout << "CorrectedAcc:" << computeAcc() << std::endl;
}

