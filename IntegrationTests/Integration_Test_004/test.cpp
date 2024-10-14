#include "pch.h"
#include "Common.h"
#include "Utils.h"
#include "experimenter.h"
#include "online_rf.h"

std::string TESTDATADIR = "C:\\Users\\HPJ\\Desktop\\Paper\\Exp\\Scene1\\subsample_cloud\\SQN_1%\\";
std::string InitialSegmentationResultPath = TESTDATADIR + "SubSample_mapd_Label_SQN_1%_block_0_1.txt";
std::string FeaturePath = TESTDATADIR + "Features\\AllFeatures.txt";
std::string VehicleFeaturePath = TESTDATADIR + "Features\\VehicleFeatures.txt";
std::string GroundFeaturePath = TESTDATADIR + "Features\\GroundFeatures.txt";
std::string BuildingFeaturePath = TESTDATADIR + "Features\\BuildingFeatures.txt";
std::string TreeFeaturePath = TESTDATADIR + "Features\\TreeFeatures.txt";

TEST(TestCaseName, TestName) {
	std::string InitialSegmentationResultPath = TESTDATADIR + "SubSample_mapd_Label_Randlanet_block_0_1.txt";
	std::string ConfFilePath = "..\\..\\OnlineLearning\\conf\\omcb.conf";
	Hyperparameters HP(ConfFilePath);
	CPointCloudTool PointCloudTool;
	std::vector<vector<double>> InitialSegmentationResult;
	std::vector<vector<double>> LastBinaryRandomForestCorrectedResult;

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
	//PointCloudTool.saveArrayFile(TESTDATADIR + "FittingResult\\GroundApproaching.txt", InitialTrainData4GroundBinary);

	////ManualResult approaching stage 1
	//vector<string> PosManualResultFilePathVector;
	//vector<string> NegManualResultFilePathVector;
	//PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\test\\1\\Positive.txt");
	//NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\test\\1\\Negative.txt");
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
	//PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\test\\1\\Result.txt", InitialSegmentationResult);

	/////******************ManualResult approaching stage 2**********************/
	///******************load the confidence of last corrected result;******************/
	//InitialSegmentationResult.clear();
	//PointCloudTool.loadOriginArray(TESTDATADIR + "Interactions\\test\\1\\Result.txt", InitialSegmentationResult);
	//PointCloudTool.deleteDuplicateConfidence(InitialSegmentationResult);
	///******************the second interaction;******************/
	//PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\test\\2\\Positive.txt");
	//NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\test\\2\\Negative.txt");
	//PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
	//DataSet GroundCorrectionTwo(NumFeatures, FirstFeatureIndex, LabelIndex);
	//GroundCorrectionTwo.load(ManualTrainData);
	//train(Model, GroundCorrectionTwo, HP);
	//Results.clear();
	//Results = test(Model, GroundDataSet, HP);
	//PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
	//PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\test\\2\\Result.txt", InitialSegmentationResult);

	///******************ManualResult approaching stage 3**********************/

	///******************load the confidence of last corrected result;******************/
	//InitialSegmentationResult.clear();
	//PointCloudTool.loadOriginArray(TESTDATADIR + "Interactions\\test\\2\\Result.txt", InitialSegmentationResult);
	//PointCloudTool.deleteDuplicateConfidence(InitialSegmentationResult);
	///******************the second interaction;******************/
	//PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\test\\3\\Positive.txt");
	//NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\test\\3\\Negative.txt");
	//PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
	//DataSet GroundCorrectionThree(NumFeatures, FirstFeatureIndex, LabelIndex);
	//GroundCorrectionThree.load(ManualTrainData);
	//train(Model, GroundCorrectionThree, HP);
	//Results.clear();
	//Results = test(Model, GroundDataSet, HP);
	//PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
	//PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\test\\3\\Result.txt", InitialSegmentationResult);

	///******************ManualResult approaching stage 4**********************/

	///******************load the confidence of last corrected result;******************/
	//InitialSegmentationResult.clear();
	//PointCloudTool.loadOriginArray(TESTDATADIR + "Interactions\\test\\3\\Result.txt", InitialSegmentationResult);
	//PointCloudTool.deleteDuplicateConfidence(InitialSegmentationResult);
	///******************the second interaction;******************/
	//PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\test\\4\\Positive.txt");
	//NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\test\\4\\Negative.txt");
	//PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
	//DataSet GroundCorrectionFour(NumFeatures, FirstFeatureIndex, LabelIndex);
	//GroundCorrectionFour.load(ManualTrainData);
	//train(Model, GroundCorrectionFour, HP);
	//Results.clear();
	//Results = test(Model, GroundDataSet, HP);
	//PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
	//PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\test\\4\\Result.txt", InitialSegmentationResult);

	//delete Model;

	///************* Tree Correction ***************/
	///************* Load Last Binary Random Forest Corrected Confidence ***************/
	//int NumFeatures = 5, FirstFeatureIndex = 6, LabelIndex = 11, ConfidenceColumnIndex = 12;
	//ESemanticCategory SemanticLabel = ESemanticCategory::Tree;
	//vector<string> PosManualResultFilePathVector;
	//vector<string> NegManualResultFilePathVector;
	//Classifier* TreeModel = nullptr;
	//PointCloudTool.loadOriginArray(TreeFeaturePath, InitialSegmentationResult);
	//LastBinaryRandomForestCorrectedResult.clear();
	//std::string LastBinaryRandomForestCorrectedResultPath= TESTDATADIR + "Interactions\\test\\4\\Result.txt";
	//PointCloudTool.loadOriginArray(LastBinaryRandomForestCorrectedResultPath, LastBinaryRandomForestCorrectedResult);
	//PointCloudTool.setLabelColumnIndex(LabelIndex);
	//int LabelIndex4LastBinaryRandomForestCorrectedResult = 10;
	//PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastBinaryRandomForestCorrectedResult, LabelIndex4LastBinaryRandomForestCorrectedResult);
	//
	///************* Fitting ***************/
	//auto InitialTrainData4TreeBinary = InitialSegmentationResult;
	//PointCloudTool.setLabelColumnIndex(LabelIndex);
	//PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4TreeBinary);
	//DataSet TreeDataSet;
	//TreeDataSet.setNumFeatures(NumFeatures);
	//TreeDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
	//TreeDataSet.setLaeblIndex(LabelIndex);
	//TreeDataSet.load(InitialTrainData4TreeBinary);
	//TreeModel = new OnlineRF(HP, TreeDataSet.m_numClasses, TreeDataSet.m_numFeatures, TreeDataSet.m_minFeatRange, TreeDataSet.m_maxFeatRange);
	//train(TreeModel, TreeDataSet, HP);
	//vector<Result> Results = test(TreeModel, TreeDataSet, HP);
	//PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4TreeBinary, Results);
	//PointCloudTool.saveArrayFile(TESTDATADIR + "FittingResult\\TreeApproaching.txt", InitialTrainData4TreeBinary);

 //   //ManualResult approaching stage 1
 //   PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\test\\5\\Positive.txt");
 //   NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\test\\5\\Negative.txt");
 //   std::vector<vector<double>> ManualTrainData;
 //   PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
 //   DataSet TreeCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
 //   TreeCorrectionOne.load(ManualTrainData);
 //   train(TreeModel, TreeCorrectionOne, HP);
 //   Results.clear();
 //   Results = test(TreeModel, TreeDataSet, HP);
 //   PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
 //   PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
 //   PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\test\\5\\Result.txt", InitialSegmentationResult);

	////ManualResult approaching stage 2
	//InitialSegmentationResult.clear();
	//PointCloudTool.loadOriginArray(TESTDATADIR + "Interactions\\test\\5\\Result.txt", InitialSegmentationResult);
	//PointCloudTool.deleteDuplicateConfidence(InitialSegmentationResult);
	///******************the second interaction;******************/
	//PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\test\\6\\Positive.txt");
	//NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\test\\6\\Negative.txt");
	//PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
	//DataSet TreeCorrectionTwo(NumFeatures, FirstFeatureIndex, LabelIndex);
	//TreeCorrectionTwo.load(ManualTrainData);
	//train(TreeModel, TreeCorrectionTwo, HP);
	//Results.clear();
	//Results = test(TreeModel, TreeDataSet, HP);
	//PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
	//PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\test\\6\\Result.txt", InitialSegmentationResult);

	////ManualResult approaching stage 3
	//InitialSegmentationResult.clear();
	//PointCloudTool.loadOriginArray(TESTDATADIR + "Interactions\\test\\6\\Result.txt", InitialSegmentationResult);
	//PointCloudTool.deleteDuplicateConfidence(InitialSegmentationResult);
	///******************the second interaction;******************/
	//PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\test\\7\\Positive.txt");
	//NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\test\\7\\Negative.txt");
	//PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
	//DataSet TreeCorrectionThree(NumFeatures, FirstFeatureIndex, LabelIndex);
	//TreeCorrectionThree.load(ManualTrainData);
	//train(TreeModel, TreeCorrectionThree, HP);
	//Results.clear();
	//Results = test(TreeModel, TreeDataSet, HP);
	//PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
	//PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\test\\7\\Result.txt", InitialSegmentationResult);

	//delete TreeModel;

	///************* Building Correction ***************/
	///************* Load Last Binary Random Forest Corrected Confidence ***************/
	//int NumFeatures = 4, FirstFeatureIndex = 6, LabelIndex = 10, ConfidenceColumnIndex = 11;
	//ESemanticCategory SemanticLabel = ESemanticCategory::Building;
	//vector<string> PosManualResultFilePathVector;
	//vector<string> NegManualResultFilePathVector;
	//Classifier* BuildingModel = nullptr;
	//PointCloudTool.loadOriginArray(BuildingFeaturePath, InitialSegmentationResult);
	//LastBinaryRandomForestCorrectedResult.clear();
	//std::string LastBinaryRandomForestCorrectedResultPath = TESTDATADIR + "Interactions\\test\\7\\Result.txt";
	//PointCloudTool.loadOriginArray(LastBinaryRandomForestCorrectedResultPath, LastBinaryRandomForestCorrectedResult);
	//PointCloudTool.setLabelColumnIndex(LabelIndex);
	//int LabelIndex4LastBinaryRandomForestCorrectedResult = 11;
	//PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastBinaryRandomForestCorrectedResult, LabelIndex4LastBinaryRandomForestCorrectedResult);

	///************* Fitting ***************/
	//auto InitialTrainData4BuildingBinary = InitialSegmentationResult;
	//PointCloudTool.setLabelColumnIndex(LabelIndex);
	//PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4BuildingBinary);
	//DataSet BuildingDataSet;
	//BuildingDataSet.setNumFeatures(NumFeatures);
	//BuildingDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
	//BuildingDataSet.setLaeblIndex(LabelIndex);
	//BuildingDataSet.load(InitialTrainData4BuildingBinary);
	//BuildingModel = new OnlineRF(HP, BuildingDataSet.m_numClasses, BuildingDataSet.m_numFeatures, BuildingDataSet.m_minFeatRange, BuildingDataSet.m_maxFeatRange);
	//train(BuildingModel, BuildingDataSet, HP);
	//vector<Result> Results = test(BuildingModel, BuildingDataSet, HP);
	//PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4BuildingBinary, Results);
	//PointCloudTool.saveArrayFile(TESTDATADIR + "FittingResult\\BuildingApproaching.txt", InitialTrainData4BuildingBinary);

	////ManualResult approaching stage 1
 //  PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\test\\8\\Positive.txt");
 //  NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\test\\8\\Negative.txt");
 //  std::vector<vector<double>> ManualTrainData;
 //  PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
 //  DataSet BuildingCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
 //  BuildingCorrectionOne.load(ManualTrainData);
 //  train(BuildingModel, BuildingCorrectionOne, HP);
 //  Results.clear();
 //  Results = test(BuildingModel, BuildingDataSet, HP);
 //  PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
 //  PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
 //  PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\test\\8\\Result.txt", InitialSegmentationResult);

 //  //ManualResult approaching stage 2
 //  InitialSegmentationResult.clear();
 //  PointCloudTool.loadOriginArray(TESTDATADIR + "Interactions\\test\\8\\Result.txt", InitialSegmentationResult);
 //  PointCloudTool.deleteDuplicateConfidence(InitialSegmentationResult);
 //  PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\test\\9\\Positive.txt");
 //  NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\test\\9\\Negative.txt");
 //  PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
 //  DataSet BuildingCorrectionTwo(NumFeatures, FirstFeatureIndex, LabelIndex);
 //  BuildingCorrectionTwo.load(ManualTrainData);
 //  train(BuildingModel, BuildingCorrectionTwo, HP);
 //  Results.clear();
 //  Results = test(BuildingModel, BuildingDataSet, HP);
 //  PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
 //  PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\test\\9\\Result.txt", InitialSegmentationResult);

 //  delete BuildingModel;

	/************* Vehicle Correction ***************/
	/************* Load Last Binary Random Forest Corrected Confidence ***************/
	int NumFeatures = 5, FirstFeatureIndex = 6, LabelIndex = 11, ConfidenceColumnIndex = 12;
	ESemanticCategory SemanticLabel = ESemanticCategory::Vehicle;
	vector<string> PosManualResultFilePathVector;
	vector<string> NegManualResultFilePathVector;
	Classifier* VehicleModel = nullptr;
	PointCloudTool.loadOriginArray(VehicleFeaturePath, InitialSegmentationResult);
	LastBinaryRandomForestCorrectedResult.clear();
	std::string LastBinaryRandomForestCorrectedResultPath = TESTDATADIR + "Interactions\\test\\9\\Result-WithVehicePoint.txt";
	PointCloudTool.loadOriginArray(LastBinaryRandomForestCorrectedResultPath, LastBinaryRandomForestCorrectedResult);
	PointCloudTool.setLabelColumnIndex(LabelIndex);
	int LabelIndex4LastBinaryRandomForestCorrectedResult = 10;
	PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastBinaryRandomForestCorrectedResult, LabelIndex4LastBinaryRandomForestCorrectedResult);

	/************* Fitting ***************/
	auto InitialTrainData4VehicleBinary = InitialSegmentationResult;
	PointCloudTool.setLabelColumnIndex(LabelIndex);
	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4VehicleBinary);
	DataSet VehicleDataSet;
	VehicleDataSet.setNumFeatures(NumFeatures);
	VehicleDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
	VehicleDataSet.setLaeblIndex(LabelIndex);
	VehicleDataSet.load(InitialTrainData4VehicleBinary);
	VehicleModel = new OnlineRF(HP, VehicleDataSet.m_numClasses, VehicleDataSet.m_numFeatures, VehicleDataSet.m_minFeatRange, VehicleDataSet.m_maxFeatRange);
	train(VehicleModel, VehicleDataSet, HP);
	vector<Result> Results = test(VehicleModel, VehicleDataSet, HP);
	PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4VehicleBinary, Results);
	PointCloudTool.saveArrayFile(TESTDATADIR + "FittingResult\\VehicleApproaching.txt", InitialTrainData4VehicleBinary);

	//ManualResult approaching stage 1
    PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\test\\10\\Positive.txt");
    NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\test\\10\\Negative.txt");
    std::vector<vector<double>> ManualTrainData;
    PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
    DataSet VehicleCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
    VehicleCorrectionOne.load(ManualTrainData);
    train(VehicleModel, VehicleCorrectionOne, HP);
	Results = test(VehicleModel, VehicleDataSet, HP);
    PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
    PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
    PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\test\\10\\Result.txt", InitialSegmentationResult);

	//ManualResult approaching stage 2
	InitialSegmentationResult.clear();
	PointCloudTool.loadOriginArray(TESTDATADIR + "Interactions\\test\\10\\Result.txt", InitialSegmentationResult);
	PointCloudTool.deleteDuplicateConfidence(InitialSegmentationResult);
	PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\test\\11\\Positive.txt");
	NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\test\\11\\Negative.txt");
	PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
	DataSet VehicleCorrectionTwo(NumFeatures, FirstFeatureIndex, LabelIndex);
	VehicleCorrectionTwo.load(ManualTrainData);
	train(VehicleModel, VehicleCorrectionTwo, HP);
	Results.clear();
	Results = test(VehicleModel, VehicleDataSet, HP);
	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
	PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\test\\11\\Result.txt", InitialSegmentationResult);

	delete VehicleModel;
}

TEST(Scene1, compute_correctd_Acc)
{
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

	std::string LastCorrectedResult = TESTDATADIR + "Interactions\\test\\11\\Result.txt";
	auto CorrectedResult = readFloatData(LastCorrectedResult);
	auto GroundthResult = readFloatData(InitialSegmentationResultPath);

	auto computeAcc = [&]()
	{
		if (CorrectedResult.size() != GroundthResult.size())
		{
			std::cout << "The CorrectedResult's size is not the smae of InitialSegmentationResult!" << std::endl;
		}

		int FittingCount = 0;
		int CorrectedLabelIndex = CorrectedResult[0].size() - 2;
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