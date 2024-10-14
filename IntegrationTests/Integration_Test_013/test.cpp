#include "pch.h"
#include "Common.h"
#include "Utils.h"
#include "experimenter.h"
#include "online_rf.h"

//TEST(Exp_BinaryRF_vs_MultiRF, MultiyRF_Scene1) {
//
//	std::string TESTDATADIR = "E:\\desktop\\Paper\\Exp\\Comparsion of Online Method\\multiRF\\Scene1\\";
//	std::string InitialSegmentationResultPath = TESTDATADIR + "SubSample_mapd_Label_Randlanet_block_0_1.txt";
//	std::string FeaturePath = TESTDATADIR + "Features\\AllFeatures.txt";
//	std::string VehicleFeaturePath = TESTDATADIR + "Features\\VehicleFeatures.txt";
//	std::string GroundFeaturePath = TESTDATADIR + "Features\\GroundFeatures.txt";
//	std::string BuildingFeaturePath = TESTDATADIR + "Features\\BuildingFeatures.txt";
//	std::string TreeFeaturePath = TESTDATADIR + "Features\\TreeFeatures.txt";
//
//	std::string ConfFilePath = "..\\..\\OnlineLearning\\conf\\omcb.conf";
//	Hyperparameters HP(ConfFilePath);
//	CPointCloudTool PointCloudTool;
//	std::vector<vector<double>> InitialSegmentationResult;
//	std::vector<vector<double>> LastBinaryRandomForestCorrectedResult;
//	int NumFeatures = 11, FirstFeatureIndex = 6, LabelIndex = 17;
//
//	/************* Ground Correction ***************/
//	PointCloudTool.loadOriginArray(FeaturePath, InitialSegmentationResult);
//	PointCloudTool.deleteDuplicateConfidence(InitialSegmentationResult);
//	auto InitialTrainData = InitialSegmentationResult;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//
//	DataSet TestDataset;
//	TestDataset.setNumFeatures(NumFeatures);
//	TestDataset.setFirstFeatureColumnIndex(FirstFeatureIndex);
//	TestDataset.setLaeblIndex(LabelIndex);
//	TestDataset.load(InitialTrainData);
//	Classifier* Model = new OnlineRF(HP, TestDataset.m_numClasses, TestDataset.m_numFeatures, TestDataset.m_minFeatRange, TestDataset.m_maxFeatRange);
//	train(Model, TestDataset, HP);
//	vector<Result> Results = test(Model, TestDataset, HP);
//	PointCloudTool.visualizeORFApproachDLResult(InitialTrainData, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "FittingResult\\Approaching.txt", InitialTrainData);
//
//
//
////cloudcompare
//// 
////     通過在綫交互
//// 	   修改訓練數據
////  
//	//ManualResult approaching stage 1
//	vector<string> PosManualResultFilePathVector;
//	vector<string> NegManualResultFilePathVector;
//	PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\1\\Positive.txt");
//	NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\1\\Negative.txt");
//
//	std::vector<vector<double>> ManualTrainData;
//	PointCloudTool.mergeMultiORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	DataSet CorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
//	CorrectionOne.load(ManualTrainData);
//	train(Model, CorrectionOne, HP);
//	Results.clear();
//	Results = test(Model, TestDataset, HP);
//
//	PointCloudTool.visualizeMultiORFApproachManualResult(InitialSegmentationResult,Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\1\\Result.txt", InitialSegmentationResult);
//
//	//ManualResult approaching stage 2
//	PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\2\\Positive.txt");
//	NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\2\\Negative.txt");
//
//	PointCloudTool.mergeMultiORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	DataSet CorrectionTwo(NumFeatures, FirstFeatureIndex, LabelIndex);
//	CorrectionTwo.load(ManualTrainData);
//	train(Model, CorrectionTwo, HP);
//	Results.clear();
//	Results = test(Model, TestDataset, HP);
//
//	PointCloudTool.visualizeMultiORFApproachManualResult(InitialSegmentationResult, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\2\\Result.txt", InitialSegmentationResult);
//
//	//ManualResult approaching stage 3
//	PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\3\\Positive.txt");
//	NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\3\\Negative.txt");
//
//	PointCloudTool.mergeMultiORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	DataSet CorrectionThree(NumFeatures, FirstFeatureIndex, LabelIndex);
//	CorrectionThree.load(ManualTrainData);
//	train(Model, CorrectionThree, HP);
//	Results.clear();
//	Results = test(Model, TestDataset, HP);
//
//	PointCloudTool.visualizeMultiORFApproachManualResult(InitialSegmentationResult, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\3\\Result.txt", InitialSegmentationResult);
//
//	//ManualResult approaching stage 4
//	//PosManualResultFilePathVector.clear();
//	//NegManualResultFilePathVector.clear();
//	PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\4\\Positive.txt");
//	NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\4\\Negative.txt");
//
//	PointCloudTool.mergeMultiORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	DataSet CorrectionFour(NumFeatures, FirstFeatureIndex, LabelIndex);
//	CorrectionFour.load(ManualTrainData);
//	train(Model, CorrectionFour, HP);
//	Results.clear();
//	Results = test(Model, TestDataset, HP);
//
//	PointCloudTool.visualizeMultiORFApproachManualResult(InitialSegmentationResult, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\4\\Result.txt", InitialSegmentationResult);
//
//	//ManualResult approaching stage 5
//	PosManualResultFilePathVector.clear();
//	NegManualResultFilePathVector.clear();
//	PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\5\\Positive.txt");
//	NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\5\\Negative.txt");
//
//	PointCloudTool.mergeMultiORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	DataSet CorrectionFive(NumFeatures, FirstFeatureIndex, LabelIndex);
//	CorrectionFive.load(ManualTrainData);
//	train(Model, CorrectionFive, HP);
//	Results.clear();
//	Results = test(Model, TestDataset, HP);
//
//	PointCloudTool.visualizeMultiORFApproachManualResult(InitialSegmentationResult, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\5\\Result.txt", InitialSegmentationResult);
//
//	delete Model;
//}

TEST(Exp_BinaryRF_vs_MultiRF, BinaryRF_Scene1) {
	std::string TESTDATADIR = "D:\\binaryRF\\Scene1\\";
	std::string InitialSegmentationResultPath = TESTDATADIR + "subsample_mapd_label_randlanet_block_0_1.txt";
	std::string FeaturePath = TESTDATADIR + "Features\\Allfeatures.txt";
	std::string VehicleFeaturePath = TESTDATADIR + "Features\\Vehiclefeatures.txt";
	std::string GroundFeaturePath = TESTDATADIR + "Features\\GroundFeatures.txt";
	std::string BuildingFeaturePath = TESTDATADIR + "Features\\BuildingFeatures.txt";
	std::string TreeFeaturePath = TESTDATADIR + "Features\\TreeFeatures.txt";

	std::string ConfFilePath = "..\\..\\OnlineLearning\\conf\\omcb.conf";
	Hyperparameters HP(ConfFilePath);
	CPointCloudTool PointCloudTool;
	std::vector<vector<double>> InitialSegmentationResult;
	std::vector<vector<double>> LastBinaryRandomForestCorrectedResult;

	/************* Ground Correction ***************/
	ESemanticCategory SemanticLabel = ESemanticCategory::Ground;
	int NumFeatures = 4, FirstFeatureIndex = 6, LabelIndex = 10;
	PointCloudTool.loadOriginArray(GroundFeaturePath, InitialSegmentationResult);
	auto InitialTrainData4GroundBinary = InitialSegmentationResult;
	PointCloudTool.setLabelColumnIndex(LabelIndex);
	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4GroundBinary);
	DataSet GroundDataSet;
	GroundDataSet.setNumFeatures(NumFeatures);
	GroundDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
	GroundDataSet.setLaeblIndex(LabelIndex);
	GroundDataSet.load(InitialTrainData4GroundBinary);
	Classifier* GroundModel = new OnlineRF(HP, GroundDataSet.m_numClasses, GroundDataSet.m_numFeatures, GroundDataSet.m_minFeatRange, GroundDataSet.m_maxFeatRange);
	train(GroundModel, GroundDataSet, HP);
	vector<Result> Results = test(GroundModel, GroundDataSet, HP);
	PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4GroundBinary, Results);
	PointCloudTool.saveArrayFile(TESTDATADIR + "FittingResult\\GroundApproaching.txt", InitialTrainData4GroundBinary);

	int ConfidenceColumnIndex = 11;
	/*ManualResult approaching stage 1*/
	vector<string> PosManualResultFilePathVector;
	vector<string> NegManualResultFilePathVector;
	PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\1\\Positive.txt");
	NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\1\\Negative.txt");
	std::vector<vector<double>> ManualTrainData;
	PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
	DataSet GroundCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
	GroundCorrectionOne.load(ManualTrainData);
	train(GroundModel, GroundCorrectionOne, HP);
	Results.clear();
	Results = test(GroundModel, GroundDataSet, HP);
	PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
	PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\1\\Result.txt", InitialSegmentationResult);

	delete GroundModel;

	///************* Vehicle Correction ***************/
	///************* Load Last Binary Random Forest Corrected Confidence ***************/
	//int NumFeatures = 3, FirstFeatureIndex = 6, LabelIndex = 9, ConfidenceColumnIndex = 10;
	//ESemanticCategory SemanticLabel = ESemanticCategory::Vehicle;
	//vector<string> PosManualResultFilePathVector;
	//vector<string> NegManualResultFilePathVector;
	//Classifier* VehicleModel = nullptr;
	//PointCloudTool.loadOriginArray(VehicleFeaturePath, InitialSegmentationResult);
	//LastBinaryRandomForestCorrectedResult.clear();
	//std::string LastBinaryRandomForestCorrectedResultPath = TESTDATADIR + "Interactions\\1\\Result.txt";
	//PointCloudTool.loadOriginArray(LastBinaryRandomForestCorrectedResultPath, LastBinaryRandomForestCorrectedResult);
	//PointCloudTool.setLabelColumnIndex(LabelIndex);
	//int LabelIndex4LastBinaryRandomForestCorrectedResult = 10;
	//PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastBinaryRandomForestCorrectedResult, LabelIndex4LastBinaryRandomForestCorrectedResult);

	///************* Fitting ***************/
	//auto InitialTrainData4VehicleBinary = InitialSegmentationResult;
	//PointCloudTool.setLabelColumnIndex(LabelIndex);
	//PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4VehicleBinary);
	//DataSet VehicleDataSet;
	//VehicleDataSet.setNumFeatures(NumFeatures);
	//VehicleDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
	//VehicleDataSet.setLaeblIndex(LabelIndex);
	//VehicleDataSet.load(InitialTrainData4VehicleBinary);
	//VehicleModel = new OnlineRF(HP, VehicleDataSet.m_numClasses, VehicleDataSet.m_numFeatures, VehicleDataSet.m_minFeatRange, VehicleDataSet.m_maxFeatRange);
	//train(VehicleModel, VehicleDataSet, HP);
	//vector<Result> Results = test(VehicleModel, VehicleDataSet, HP);
	//PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4VehicleBinary, Results);
	//PointCloudTool.saveArrayFile(TESTDATADIR + "FittingResult\\VehicleApproaching.txt", InitialTrainData4VehicleBinary);

	////ManualResult approaching stage 1
	//PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\2\\Positive.txt");
	//NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\2\\Negative.txt");
	//std::vector<vector<double>> ManualTrainData;
	//PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
	//DataSet VehicleCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
	//VehicleCorrectionOne.load(ManualTrainData);
	//train(VehicleModel, VehicleCorrectionOne, HP);
	//Results = test(VehicleModel, VehicleDataSet, HP);
	//PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
	//PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
	//PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\2\\Result.txt", InitialSegmentationResult);

	////ManualResult approaching stage 2
	//InitialSegmentationResult.clear();
	//PointCloudTool.loadOriginArray(TESTDATADIR + "Interactions\\2\\Result.txt", InitialSegmentationResult);
	//PointCloudTool.deleteDuplicateConfidence(InitialSegmentationResult);
	//PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\3\\Positive.txt");
	//NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\3\\Negative.txt");
	//PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
	//DataSet VehicleCorrectionTwo(NumFeatures, FirstFeatureIndex, LabelIndex);
	//VehicleCorrectionTwo.load(ManualTrainData);
	//train(VehicleModel, VehicleCorrectionTwo, HP);
	//Results.clear();
	//Results = test(VehicleModel, VehicleDataSet, HP);
	//PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
	//PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\3\\Result.txt", InitialSegmentationResult);

	//delete VehicleModel;

	///************* Ground Correction ***************/
	///************* Load Last Binary Random Forest Corrected Confidence ***************/
	//int NumFeatures = 4, FirstFeatureIndex = 6, LabelIndex = 10, ConfidenceColumnIndex = 11;
	//ESemanticCategory SemanticLabel = ESemanticCategory::Ground;
	//vector<string> PosManualResultFilePathVector;
	//vector<string> NegManualResultFilePathVector;
	//Classifier* GroundModel = nullptr;
	//PointCloudTool.loadOriginArray(GroundFeaturePath, InitialSegmentationResult);
	//LastBinaryRandomForestCorrectedResult.clear();
	//std::string LastBinaryRandomForestCorrectedResultPath = TESTDATADIR + "Interactions\\3\\Result.txt";
	//PointCloudTool.loadOriginArray(LastBinaryRandomForestCorrectedResultPath, LastBinaryRandomForestCorrectedResult);
	//PointCloudTool.setLabelColumnIndex(LabelIndex);
	//int LabelIndex4LastBinaryRandomForestCorrectedResult = 9;
	//PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastBinaryRandomForestCorrectedResult, LabelIndex4LastBinaryRandomForestCorrectedResult);

	///************* Fitting ***************/
	//auto InitialTrainData4GroundBinary = InitialSegmentationResult;
	//PointCloudTool.setLabelColumnIndex(LabelIndex);
	//PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4GroundBinary);
	//DataSet GroundDataSet;
	//GroundDataSet.setNumFeatures(NumFeatures);
	//GroundDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
	//GroundDataSet.setLaeblIndex(LabelIndex);
	//GroundDataSet.load(InitialTrainData4GroundBinary);
	//GroundModel = new OnlineRF(HP, GroundDataSet.m_numClasses, GroundDataSet.m_numFeatures, GroundDataSet.m_minFeatRange, GroundDataSet.m_maxFeatRange);
	//train(GroundModel, GroundDataSet, HP);
	//vector<Result> Results = test(GroundModel, GroundDataSet, HP);
	//PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4GroundBinary, Results);
	//PointCloudTool.saveArrayFile(TESTDATADIR + "FittingResult\\TheSecondGroundApproaching.txt", InitialTrainData4GroundBinary);

	////ManualResult approaching stage 1
	//PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\4\\Positive.txt");
	//NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\4\\Negative.txt");
	//std::vector<vector<double>> ManualTrainData;
	//PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
	//DataSet GroundCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
	//GroundCorrectionOne.load(ManualTrainData);
	//train(GroundModel, GroundCorrectionOne, HP);
	//Results = test(GroundModel, GroundDataSet, HP);
	//PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
	//PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
	//PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\4\\Result.txt", InitialSegmentationResult);

	//delete GroundModel;

	///************* Building Correction ***************/
	///************* Load Last Binary Random Forest Corrected Confidence ***************/
	//int NumFeatures = 4, FirstFeatureIndex = 6, LabelIndex = 10, ConfidenceColumnIndex = 11;
	//ESemanticCategory SemanticLabel = ESemanticCategory::Building;
	//vector<string> PosManualResultFilePathVector;
	//vector<string> NegManualResultFilePathVector;
	//Classifier* BuildingModel = nullptr;
	//PointCloudTool.loadOriginArray(BuildingFeaturePath, InitialSegmentationResult);
	//LastBinaryRandomForestCorrectedResult.clear();
	//std::string LastBinaryRandomForestCorrectedResultPath = TESTDATADIR + "Interactions\\4\\Result.txt";
	//PointCloudTool.loadOriginArray(LastBinaryRandomForestCorrectedResultPath, LastBinaryRandomForestCorrectedResult);
	//PointCloudTool.setLabelColumnIndex(LabelIndex);
	//int LabelIndex4LastBinaryRandomForestCorrectedResult = 10;
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
	//PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\5\\Positive.txt");
	//NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\5\\Negative.txt");
	//std::vector<vector<double>> ManualTrainData;
	//PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
	//DataSet BuildingCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
	//BuildingCorrectionOne.load(ManualTrainData);
	//train(BuildingModel, BuildingCorrectionOne, HP);
	//Results = test(BuildingModel, BuildingDataSet, HP);
	//PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
	//PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
	//PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\5\\Result.txt", InitialSegmentationResult);

	////ManualResult approaching stage 2
	//PosManualResultFilePathVector.clear();
	//NegManualResultFilePathVector.clear();
	//PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\6\\Positive.txt");
	//NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\6\\Negative.txt");
	//PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
	//DataSet BuildingCorrectionTwo(NumFeatures, FirstFeatureIndex, LabelIndex);
	//BuildingCorrectionTwo.load(ManualTrainData);
	//train(BuildingModel, BuildingCorrectionTwo, HP);
	//Results = test(BuildingModel, BuildingDataSet, HP);
	//PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
	//PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
	//PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\6\\Result.txt", InitialSegmentationResult);

	////ManualResult approaching stage 3
	//PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\7\\Positive.txt");
	//NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\7\\Negative.txt");
	//PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
	//DataSet BuildingCorrectionThree(NumFeatures, FirstFeatureIndex, LabelIndex);
	//BuildingCorrectionThree.load(ManualTrainData);
	//train(BuildingModel, BuildingCorrectionThree, HP);
	//Results = test(BuildingModel, BuildingDataSet, HP);
	//PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
	//PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
	//PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\7\\Result.txt", InitialSegmentationResult);

	//delete BuildingModel;

	///************* Tree Correction ***************/
	///************* Load Last Binary Random Forest Corrected Confidence ***************/
	//int NumFeatures = 5, FirstFeatureIndex = 6, LabelIndex = 11, ConfidenceColumnIndex = 12;
	//ESemanticCategory SemanticLabel = ESemanticCategory::Tree;
	//vector<string> PosManualResultFilePathVector;
	//vector<string> NegManualResultFilePathVector;
	//Classifier* TreeModel = nullptr;
	//PointCloudTool.loadOriginArray(TreeFeaturePath, InitialSegmentationResult);
	//LastBinaryRandomForestCorrectedResult.clear();
	//std::string LastBinaryRandomForestCorrectedResultPath = TESTDATADIR + "Interactions\\7\\Result.txt";
	//PointCloudTool.loadOriginArray(LastBinaryRandomForestCorrectedResultPath, LastBinaryRandomForestCorrectedResult);
	//PointCloudTool.setLabelColumnIndex(LabelIndex);
	//int LabelIndex4LastBinaryRandomForestCorrectedResult = 10;
	//PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastBinaryRandomForestCorrectedResult, LabelIndex4LastBinaryRandomForestCorrectedResult);

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

	////ManualResult approaching stage 1
	//PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\8\\Positive.txt");
	//NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\8\\Negative.txt");
	//std::vector<vector<double>> ManualTrainData;
	//PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
	//DataSet TreeCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
	//TreeCorrectionOne.load(ManualTrainData);
	//train(TreeModel, TreeCorrectionOne, HP);
	//Results = test(TreeModel, TreeDataSet, HP);
	//PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
	//PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
	//PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\8\\Result.txt", InitialSegmentationResult);

	//delete TreeModel;
}

//TEST(Exp_BinaryRF_vs_MultiRF, BinaryRF_Scene2) {
//	std::string TESTDATADIR = "C:\\Users\\HPJ\\Desktop\\Paper\\Exp\\binaryRF_vs_multiRF\\binaryRF\\Scene2\\";
//	std::string InitialSegmentationResultPath = TESTDATADIR + "SubSample_mapd_Label_Randlanet_block_1_0.txt";
//	std::string FeaturePath = TESTDATADIR + "Features\\AllFeatures.txt";
//	std::string VehicleFeaturePath = TESTDATADIR + "Features\\VehicleFeatures.txt";
//	std::string GroundFeaturePath = TESTDATADIR + "Features\\GroundFeatures.txt";
//	std::string BuildingFeaturePath = TESTDATADIR + "Features\\BuildingFeatures.txt";
//	std::string TreeFeaturePath = TESTDATADIR + "Features\\TreeFeatures.txt";
//
//	std::string ConfFilePath = "..\\..\\OnlineLearning\\conf\\omcb.conf";
//	Hyperparameters HP(ConfFilePath);
//	CPointCloudTool PointCloudTool;
//	std::vector<vector<double>> InitialSegmentationResult;
//	std::vector<vector<double>> LastBinaryRandomForestCorrectedResult;
//
//	///************* Vehicle Correction ***************/
//	///************* Load Last Binary Random Forest Corrected Confidence ***************/
//	//int NumFeatures = 3, FirstFeatureIndex = 6, LabelIndex = 9, ConfidenceColumnIndex = 10;
//	//ESemanticCategory SemanticLabel = ESemanticCategory::Vehicle;
//	//vector<string> PosManualResultFilePathVector;
//	//vector<string> NegManualResultFilePathVector;
//	//Classifier* VehicleModel = nullptr;
//	//PointCloudTool.loadOriginArray(VehicleFeaturePath, InitialSegmentationResult);
//
//	///************* Fitting ***************/
//	//auto InitialTrainData4VehicleBinary = InitialSegmentationResult;
//	//PointCloudTool.setLabelColumnIndex(LabelIndex);
//	//PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4VehicleBinary);
//	//DataSet VehicleDataSet;
//	//VehicleDataSet.setNumFeatures(NumFeatures);
//	//VehicleDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
//	//VehicleDataSet.setLaeblIndex(LabelIndex);
//	//VehicleDataSet.load(InitialTrainData4VehicleBinary);
//	//VehicleModel = new OnlineRF(HP, VehicleDataSet.m_numClasses, VehicleDataSet.m_numFeatures, VehicleDataSet.m_minFeatRange, VehicleDataSet.m_maxFeatRange);
//	//train(VehicleModel, VehicleDataSet, HP);
//	//vector<Result> Results = test(VehicleModel, VehicleDataSet, HP);
//	//PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4VehicleBinary, Results);
//	//PointCloudTool.saveArrayFile(TESTDATADIR + "FittingResult\\VehicleApproaching.txt", InitialTrainData4VehicleBinary);
//
//	////ManualResult approaching stage 1
//	//PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\1\\Positive.txt");
//	//NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\1\\Negative.txt");
//	//std::vector<vector<double>> ManualTrainData;
//	//PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	//DataSet VehicleCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
//	//VehicleCorrectionOne.load(ManualTrainData);
//	//train(VehicleModel, VehicleCorrectionOne, HP);
//	//Results = test(VehicleModel, VehicleDataSet, HP);
//	//PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
//	//PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	//PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\1\\Result.txt", InitialSegmentationResult);
//
//	////ManualResult approaching stage 1
//	//PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\2\\Positive.txt");
//	//NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\2\\Negative.txt");
//	//PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	//DataSet VehicleCorrectionTwo(NumFeatures, FirstFeatureIndex, LabelIndex);
//	//VehicleCorrectionTwo.load(ManualTrainData);
//	//train(VehicleModel, VehicleCorrectionTwo, HP);
//	//Results = test(VehicleModel, VehicleDataSet, HP);
//	//PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
//	//PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	//PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\2\\Result.txt", InitialSegmentationResult);
//
//	//delete VehicleModel;
//
//	///************* Ground Correction ***************/
//	//ESemanticCategory SemanticLabel = ESemanticCategory::Ground;
//	//int NumFeatures = 4, FirstFeatureIndex = 6, LabelIndex = 10, ConfidenceColumnIndex = 11;
//	//vector<string> PosManualResultFilePathVector;
//	//vector<string> NegManualResultFilePathVector;
//	//Classifier* GroundModel = nullptr;
//	//PointCloudTool.loadOriginArray(GroundFeaturePath, InitialSegmentationResult);
//	//LastBinaryRandomForestCorrectedResult.clear();
//	//std::string LastBinaryRandomForestCorrectedResultPath = TESTDATADIR + "Interactions\\2\\Result.txt";
//	//PointCloudTool.loadOriginArray(LastBinaryRandomForestCorrectedResultPath, LastBinaryRandomForestCorrectedResult);
//	//PointCloudTool.setLabelColumnIndex(LabelIndex);
//	//int LabelIndex4LastBinaryRandomForestCorrectedResult = 9;
//	//PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastBinaryRandomForestCorrectedResult, LabelIndex4LastBinaryRandomForestCorrectedResult);
//
//	///************* Fitting ***************/
//	//auto InitialTrainData4GroundBinary = InitialSegmentationResult;
//	//PointCloudTool.setLabelColumnIndex(LabelIndex);
//	//PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4GroundBinary);
//	//DataSet GroundDataSet;
//	//GroundDataSet.setNumFeatures(NumFeatures);
//	//GroundDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
//	//GroundDataSet.setLaeblIndex(LabelIndex);
//	//GroundDataSet.load(InitialTrainData4GroundBinary);
//	//GroundModel = new OnlineRF(HP, GroundDataSet.m_numClasses, GroundDataSet.m_numFeatures, GroundDataSet.m_minFeatRange, GroundDataSet.m_maxFeatRange);
//	//train(GroundModel, GroundDataSet, HP);
//	//vector<Result> Results = test(GroundModel, GroundDataSet, HP);
//	//PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4GroundBinary, Results);
//	//PointCloudTool.saveArrayFile(TESTDATADIR + "FittingResult\\GroundApproaching.txt", InitialTrainData4GroundBinary);
//
//	///*ManualResult approaching stage 1*/
//	//PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\3\\Positive.txt");
//	//NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\3\\Negative.txt");
//	//std::vector<vector<double>> ManualTrainData;
//	//PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	//DataSet GroundCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
//	//GroundCorrectionOne.load(ManualTrainData);
//	//train(GroundModel, GroundCorrectionOne, HP);
//	//Results.clear();
//	//Results = test(GroundModel, GroundDataSet, HP);
//	//PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
//	//PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	//PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\3\\Result.txt", InitialSegmentationResult);
//
//	///*ManualResult approaching stage 2*/
//	//PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\4\\Positive.txt");
//	//NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\4\\Negative.txt");
//	//PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	//DataSet GroundCorrectionTwo(NumFeatures, FirstFeatureIndex, LabelIndex);
//	//GroundCorrectionTwo.load(ManualTrainData);
//	//train(GroundModel, GroundCorrectionTwo, HP);
//	//Results.clear();
//	//Results = test(GroundModel, GroundDataSet, HP);
//	//PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
//	//PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	//PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\4\\Result.txt", InitialSegmentationResult);
//
//	//delete GroundModel;
//
//	///************* Building Correction ***************/
//	///************* Load Last Binary Random Forest Corrected Confidence ***************/
//	//int NumFeatures = 4, FirstFeatureIndex = 6, LabelIndex = 10, ConfidenceColumnIndex = 11;
//	//ESemanticCategory SemanticLabel = ESemanticCategory::Building;
//	//vector<string> PosManualResultFilePathVector;
//	//vector<string> NegManualResultFilePathVector;
//	//Classifier* BuildingModel = nullptr;
//	//PointCloudTool.loadOriginArray(BuildingFeaturePath, InitialSegmentationResult);
//	//LastBinaryRandomForestCorrectedResult.clear();
//	//std::string LastBinaryRandomForestCorrectedResultPath = TESTDATADIR + "Interactions\\4\\Result.txt";
//	//PointCloudTool.loadOriginArray(LastBinaryRandomForestCorrectedResultPath, LastBinaryRandomForestCorrectedResult);
//	//PointCloudTool.setLabelColumnIndex(LabelIndex);
//	//int LabelIndex4LastBinaryRandomForestCorrectedResult = 10;
//	//PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastBinaryRandomForestCorrectedResult, LabelIndex4LastBinaryRandomForestCorrectedResult);
//
//	///************* Fitting ***************/
//	//auto InitialTrainData4BuildingBinary = InitialSegmentationResult;
//	//PointCloudTool.setLabelColumnIndex(LabelIndex);
//	//PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4BuildingBinary);
//	//DataSet BuildingDataSet;
//	//BuildingDataSet.setNumFeatures(NumFeatures);
//	//BuildingDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
//	//BuildingDataSet.setLaeblIndex(LabelIndex);
//	//BuildingDataSet.load(InitialTrainData4BuildingBinary);
//	//BuildingModel = new OnlineRF(HP, BuildingDataSet.m_numClasses, BuildingDataSet.m_numFeatures, BuildingDataSet.m_minFeatRange, BuildingDataSet.m_maxFeatRange);
//	//train(BuildingModel, BuildingDataSet, HP);
//	//vector<Result> Results = test(BuildingModel, BuildingDataSet, HP);
//	//PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4BuildingBinary, Results);
//	//PointCloudTool.saveArrayFile(TESTDATADIR + "FittingResult\\BuildingApproaching.txt", InitialTrainData4BuildingBinary);
//
//	////ManualResult approaching stage 1
//	//PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\5\\Positive.txt");
//	//NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\5\\Negative.txt");
//	//std::vector<vector<double>> ManualTrainData;
//	//PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	//DataSet BuildingCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
//	//BuildingCorrectionOne.load(ManualTrainData);
//	//train(BuildingModel, BuildingCorrectionOne, HP);
//	//Results = test(BuildingModel, BuildingDataSet, HP);
//	//PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
//	//PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	//PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\5\\Result.txt", InitialSegmentationResult);
//
//	////ManualResult approaching stage 2
//	//PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\6\\Positive.txt");
//	//NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\6\\Negative.txt");
//	//PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	//DataSet BuildingCorrectionTwo(NumFeatures, FirstFeatureIndex, LabelIndex);
//	//BuildingCorrectionTwo.load(ManualTrainData);
//	//train(BuildingModel, BuildingCorrectionTwo, HP);
//	//Results = test(BuildingModel, BuildingDataSet, HP);
//	//PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
//	//PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	//PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\6\\Result.txt", InitialSegmentationResult);
//
//	//delete BuildingModel;
//
//	///************* Tree Correction ***************/
//	///************* Load Last Binary Random Forest Corrected Confidence ***************/
//	//int NumFeatures = 5, FirstFeatureIndex = 6, LabelIndex = 11, ConfidenceColumnIndex = 12;
//	//ESemanticCategory SemanticLabel = ESemanticCategory::Tree;
//	//vector<string> PosManualResultFilePathVector;
//	//vector<string> NegManualResultFilePathVector;
//	//Classifier* TreeModel = nullptr;
//	//PointCloudTool.loadOriginArray(TreeFeaturePath, InitialSegmentationResult);
//	//LastBinaryRandomForestCorrectedResult.clear();
//	//std::string LastBinaryRandomForestCorrectedResultPath = TESTDATADIR + "Interactions\\6\\Result.txt";
//	//PointCloudTool.loadOriginArray(LastBinaryRandomForestCorrectedResultPath, LastBinaryRandomForestCorrectedResult);
//	//PointCloudTool.setLabelColumnIndex(LabelIndex);
//	//int LabelIndex4LastBinaryRandomForestCorrectedResult = 10;
//	//PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastBinaryRandomForestCorrectedResult, LabelIndex4LastBinaryRandomForestCorrectedResult);
//
//	///************* Fitting ***************/
//	//auto InitialTrainData4TreeBinary = InitialSegmentationResult;
//	//PointCloudTool.setLabelColumnIndex(LabelIndex);
//	//PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4TreeBinary);
//	//DataSet TreeDataSet;
//	//TreeDataSet.setNumFeatures(NumFeatures);
//	//TreeDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
//	//TreeDataSet.setLaeblIndex(LabelIndex);
//	//TreeDataSet.load(InitialTrainData4TreeBinary);
//	//TreeModel = new OnlineRF(HP, TreeDataSet.m_numClasses, TreeDataSet.m_numFeatures, TreeDataSet.m_minFeatRange, TreeDataSet.m_maxFeatRange);
//	//train(TreeModel, TreeDataSet, HP);
//	//vector<Result> Results = test(TreeModel, TreeDataSet, HP);
//	//PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4TreeBinary, Results);
//	//PointCloudTool.saveArrayFile(TESTDATADIR + "FittingResult\\TreeApproaching.txt", InitialTrainData4TreeBinary);
//
//	////ManualResult approaching stage 1
//	//PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\7\\Positive.txt");
//	//NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\7\\Negative.txt");
//	//std::vector<vector<double>> ManualTrainData;
//	//PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	//DataSet TreeCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
//	//TreeCorrectionOne.load(ManualTrainData);
//	//train(TreeModel, TreeCorrectionOne, HP);
//	//Results = test(TreeModel, TreeDataSet, HP);
//	//PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
//	//PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	//PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\7\\Result.txt", InitialSegmentationResult);
//
//	//delete TreeModel;
//}

//TEST(Exp_BinaryRF_vs_MultiRF, BinaryRF_Scene3) {
//	std::string TESTDATADIR = "C:\\Users\\HPJ\\Desktop\\Paper\\Exp\\binaryRF_vs_multiRF\\binaryRF\\Scene3\\";
//	std::string InitialSegmentationResultPath = TESTDATADIR + "SubSample_mapd_Label_Randlanet_block_0_2.txt";
//	std::string FeaturePath = TESTDATADIR + "Features\\AllFeatures.txt";
//	std::string VehicleFeaturePath = TESTDATADIR + "Features\\VehicleFeatures.txt";
//	std::string GroundFeaturePath = TESTDATADIR + "Features\\GroundFeatures.txt";
//	std::string BuildingFeaturePath = TESTDATADIR + "Features\\BuildingFeatures.txt";
//	std::string TreeFeaturePath = TESTDATADIR + "Features\\TreeFeatures.txt";
//
//	std::string ConfFilePath = "..\\..\\OnlineLearning\\conf\\omcb.conf";
//	Hyperparameters HP(ConfFilePath);
//	CPointCloudTool PointCloudTool;
//	std::vector<vector<double>> InitialSegmentationResult;
//	std::vector<vector<double>> LastBinaryRandomForestCorrectedResult;
//
//	/************* Vehicle Correction ***************/
//	/************* Load Last Binary Random Forest Corrected Confidence ***************/
//	int NumFeatures = 3, FirstFeatureIndex = 6, LabelIndex = 9, ConfidenceColumnIndex = 10;
//	ESemanticCategory SemanticLabel = ESemanticCategory::Vehicle;
//	vector<string> PosManualResultFilePathVector;
//	vector<string> NegManualResultFilePathVector;
//	Classifier* VehicleModel = nullptr;
//	PointCloudTool.loadOriginArray(VehicleFeaturePath, InitialSegmentationResult);
//
//	/************* Fitting ***************/
//	auto InitialTrainData4VehicleBinary = InitialSegmentationResult;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4VehicleBinary);
//	DataSet VehicleDataSet;
//	VehicleDataSet.setNumFeatures(NumFeatures);
//	VehicleDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
//	VehicleDataSet.setLaeblIndex(LabelIndex);
//	VehicleDataSet.load(InitialTrainData4VehicleBinary);
//	VehicleModel = new OnlineRF(HP, VehicleDataSet.m_numClasses, VehicleDataSet.m_numFeatures, VehicleDataSet.m_minFeatRange, VehicleDataSet.m_maxFeatRange);
//	train(VehicleModel, VehicleDataSet, HP);
//	vector<Result> Results = test(VehicleModel, VehicleDataSet, HP);
//	PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4VehicleBinary, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "FittingResult\\VehicleApproaching.txt", InitialTrainData4VehicleBinary);
//
//	////ManualResult approaching stage 1
//	//PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\1\\Positive.txt");
//	//NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\1\\Negative.txt");
//	//std::vector<vector<double>> ManualTrainData;
//	//PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	//DataSet VehicleCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
//	//VehicleCorrectionOne.load(ManualTrainData);
//	//train(VehicleModel, VehicleCorrectionOne, HP);
//	//Results = test(VehicleModel, VehicleDataSet, HP);
//	//PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
//	//PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	//PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\1\\Result.txt", InitialSegmentationResult);
//
//	//delete VehicleModel;
//
//	///************* Ground Correction ***************/
//	//ESemanticCategory SemanticLabel = ESemanticCategory::Ground;
//	//int NumFeatures = 4, FirstFeatureIndex = 6, LabelIndex = 10, ConfidenceColumnIndex = 11;
//	//vector<string> PosManualResultFilePathVector;
//	//vector<string> NegManualResultFilePathVector;
//	//Classifier* GroundModel = nullptr;
//	//PointCloudTool.loadOriginArray(GroundFeaturePath, InitialSegmentationResult);
//	//LastBinaryRandomForestCorrectedResult.clear();
//	//std::string LastBinaryRandomForestCorrectedResultPath = TESTDATADIR + "Interactions\\1\\Result.txt";
//	//PointCloudTool.loadOriginArray(LastBinaryRandomForestCorrectedResultPath, LastBinaryRandomForestCorrectedResult);
//	//PointCloudTool.setLabelColumnIndex(LabelIndex);
//	//int LabelIndex4LastBinaryRandomForestCorrectedResult = 9;
//	//PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastBinaryRandomForestCorrectedResult, LabelIndex4LastBinaryRandomForestCorrectedResult);
//
//	///************* Fitting ***************/
//	//auto InitialTrainData4GroundBinary = InitialSegmentationResult;
//	//PointCloudTool.setLabelColumnIndex(LabelIndex);
//	//PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4GroundBinary);
//	//DataSet GroundDataSet;
//	//GroundDataSet.setNumFeatures(NumFeatures);
//	//GroundDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
//	//GroundDataSet.setLaeblIndex(LabelIndex);
//	//GroundDataSet.load(InitialTrainData4GroundBinary);
//	//GroundModel = new OnlineRF(HP, GroundDataSet.m_numClasses, GroundDataSet.m_numFeatures, GroundDataSet.m_minFeatRange, GroundDataSet.m_maxFeatRange);
//	//train(GroundModel, GroundDataSet, HP);
//	//vector<Result> Results = test(GroundModel, GroundDataSet, HP);
//	//PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4GroundBinary, Results);
//	//PointCloudTool.saveArrayFile(TESTDATADIR + "FittingResult\\GroundApproaching.txt", InitialTrainData4GroundBinary);
//
//	///*ManualResult approaching stage 1*/
//	//PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\2\\Positive.txt");
//	//NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\2\\Negative.txt");
//	//std::vector<vector<double>> ManualTrainData;
//	//PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	//DataSet GroundCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
//	//GroundCorrectionOne.load(ManualTrainData);
//	//train(GroundModel, GroundCorrectionOne, HP);
//	//Results.clear();
//	//Results = test(GroundModel, GroundDataSet, HP);
//	//PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
//	//PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	//PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\2\\Result.txt", InitialSegmentationResult);
//
//	//delete GroundModel;
//
//	///************* Building Correction ***************/
//	///************* Load Last Binary Random Forest Corrected Confidence ***************/
//	//int NumFeatures = 4, FirstFeatureIndex = 6, LabelIndex = 10, ConfidenceColumnIndex = 11;
//	//ESemanticCategory SemanticLabel = ESemanticCategory::Building;
//	//vector<string> PosManualResultFilePathVector;
//	//vector<string> NegManualResultFilePathVector;
//	//Classifier* BuildingModel = nullptr;
//	//PointCloudTool.loadOriginArray(BuildingFeaturePath, InitialSegmentationResult);
//	//LastBinaryRandomForestCorrectedResult.clear();
//	//std::string LastBinaryRandomForestCorrectedResultPath = TESTDATADIR + "Interactions\\2\\Result.txt";
//	//PointCloudTool.loadOriginArray(LastBinaryRandomForestCorrectedResultPath, LastBinaryRandomForestCorrectedResult);
//	//PointCloudTool.setLabelColumnIndex(LabelIndex);
//	//int LabelIndex4LastBinaryRandomForestCorrectedResult = 10;
//	//PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastBinaryRandomForestCorrectedResult, LabelIndex4LastBinaryRandomForestCorrectedResult);
//
//	///************* Fitting ***************/
//	//auto InitialTrainData4BuildingBinary = InitialSegmentationResult;
//	//PointCloudTool.setLabelColumnIndex(LabelIndex);
//	//PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4BuildingBinary);
//	//DataSet BuildingDataSet;
//	//BuildingDataSet.setNumFeatures(NumFeatures);
//	//BuildingDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
//	//BuildingDataSet.setLaeblIndex(LabelIndex);
//	//BuildingDataSet.load(InitialTrainData4BuildingBinary);
//	//BuildingModel = new OnlineRF(HP, BuildingDataSet.m_numClasses, BuildingDataSet.m_numFeatures, BuildingDataSet.m_minFeatRange, BuildingDataSet.m_maxFeatRange);
//	//train(BuildingModel, BuildingDataSet, HP);
//	//vector<Result> Results = test(BuildingModel, BuildingDataSet, HP);
//	//PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4BuildingBinary, Results);
//	//PointCloudTool.saveArrayFile(TESTDATADIR + "FittingResult\\BuildingApproaching.txt", InitialTrainData4BuildingBinary);
//
//	////ManualResult approaching stage 1
//	//PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\3\\Positive.txt");
//	//NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\3\\Negative.txt");
//	//std::vector<vector<double>> ManualTrainData;
//	//PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	//DataSet BuildingCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
//	//BuildingCorrectionOne.load(ManualTrainData);
//	//train(BuildingModel, BuildingCorrectionOne, HP);
//	//Results = test(BuildingModel, BuildingDataSet, HP);
//	//PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
//	//PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	//PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\3\\Result.txt", InitialSegmentationResult);
//
//	////ManualResult approaching stage 2
//	//PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\4\\Positive.txt");
//	//NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\4\\Negative.txt");
//	//PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	//DataSet BuildingCorrectionTwo(NumFeatures, FirstFeatureIndex, LabelIndex);
//	//BuildingCorrectionTwo.load(ManualTrainData);
//	//train(BuildingModel, BuildingCorrectionTwo, HP);
//	//Results = test(BuildingModel, BuildingDataSet, HP);
//	//PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
//	//PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	//PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\4\\Result.txt", InitialSegmentationResult);
//
//	////ManualResult approaching stage 3
//	//PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\5\\Positive.txt");
//	//NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\5\\Negative.txt");
//	//PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	//DataSet BuildingCorrectionThree(NumFeatures, FirstFeatureIndex, LabelIndex);
//	//BuildingCorrectionThree.load(ManualTrainData);
//	//train(BuildingModel, BuildingCorrectionThree, HP);
//	//Results = test(BuildingModel, BuildingDataSet, HP);
//	//PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
//	//PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	//PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\5\\Result.txt", InitialSegmentationResult);
//
//	//delete BuildingModel;
//}

//TEST(Exp_BinaryRF_vs_MultiRF, compute_multiRF_Acc)
//{
//	std::string TESTDATADIR = "C:\\Users\\HPJ\\Desktop\\Paper\\Exp\\binaryRF_vs_multiRF\\multiRF\\";
//	std::string InitialSegmentationResultPath = TESTDATADIR + "SubSample_mapd_Label_Randlanet_block_0_1.txt";
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
//	std::string LastCorrectedResult = TESTDATADIR + "Interactions\\5\\Result.txt";
//	auto CorrectedResult = readFloatData(LastCorrectedResult);
//	auto GroundthResult = readFloatData(InitialSegmentationResultPath);
//
//	auto computeAcc = [&]()
//	{
//		if (CorrectedResult.size() != GroundthResult.size())
//		{
//			std::cout << "The CorrectedResult's size is not the smae of InitialSegmentationResult!" << std::endl;
//		}
//
//		int FittingCount = 0;
//		int CorrectedLabelIndex = CorrectedResult[0].size() - 1;
//		int GroundthLabelIndex = GroundthResult[0].size() - 1;
//		for (int i = 0; i < CorrectedResult.size(); i++)
//		{
//			if (int(CorrectedResult[i][CorrectedLabelIndex]) == int(GroundthResult[i][GroundthLabelIndex]))
//			{
//				FittingCount++;
//			}
//		}
//
//		return (float)FittingCount / GroundthResult.size();
//	};
//
//	std::cout << "CorrectedAcc:" << computeAcc() << std::endl;
//}

//TEST(Exp_BinaryRF_vs_MultiRF, compute_binaryRF_Acc)
//{
//	std::string TESTDATADIR = "C:\\Users\\HPJ\\Desktop\\Paper\\Exp\\binaryRF_vs_multiRF\\binaryRF\\Scene3\\";
//	std::string InitialSegmentationResultPath = TESTDATADIR + "SubSample_mapd_Label_Randlanet_block_0_2.txt";
//	std::string LastCorrectedResult = TESTDATADIR + "Interactions\\5\\Result.txt";
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
//	auto CorrectedResult = readFloatData(LastCorrectedResult);
//	auto GroundthResult = readFloatData(InitialSegmentationResultPath);
//
//	auto computeAcc = [&]()
//	{
//		if (CorrectedResult.size() != GroundthResult.size())
//		{
//			std::cout << "The CorrectedResult's size is not the smae of InitialSegmentationResult!" << std::endl;
//		}
//
//		int FittingCount = 0;
//		int CorrectedLabelIndex = CorrectedResult[0].size() - 2;
//		int GroundthLabelIndex = GroundthResult[0].size() - 1;
//		for (int i = 0; i < CorrectedResult.size(); i++)
//		{
//			if (int(CorrectedResult[i][CorrectedLabelIndex]) == int(GroundthResult[i][GroundthLabelIndex]))
//			{
//				FittingCount++;
//			}
//		}
//
//		return (float)FittingCount / GroundthResult.size();
//	};
//
//	std::cout << "CorrectedAcc:" << computeAcc() << std::endl;
//}

//TEST(Exp_BinaryRF_vs_MultiRF, compute_Acc_by_map)
//{
//	auto __loadPredictedCloud = [](const std::string& vCloudTXTFilePath, PointCloudT::Ptr& voCloud)
//	{
//        std::ifstream Reader(vCloudTXTFilePath);
//        std::string Line;
//        while (std::getline(Reader, Line))
//        {
//            std::stringstream SS(Line);
//            std::string TMP;
//            PointT Point;
//            std::vector<double> String2Double;
//            while (std::getline(SS, TMP, ' '))
//            {
//                String2Double.push_back(std::stod(TMP));
//            }
//
//            Point.x = String2Double[0];
//            Point.y = String2Double[1];
//            Point.z = String2Double[2];
//            Point.r = static_cast<int>(String2Double[3]);
//            Point.g = static_cast<int>(String2Double[4]);
//            Point.b = static_cast<int>(String2Double[5]);
//            Point.a = static_cast<int>(String2Double[String2Double.size()-1]);
//            voCloud->push_back(Point);
//        }
//        Reader.close();
//	};
//
//    auto __loadGroundTruthCloud = [](const std::string& vCloudTXTFilePath, PointCloudT::Ptr& voCloud)
//    {
//        std::ifstream Reader(vCloudTXTFilePath);
//        std::string Line;
//        while (std::getline(Reader, Line))
//        {
//            std::stringstream SS(Line);
//            std::string TMP;
//            PointT Point;
//            std::vector<double> String2Double;
//            while (std::getline(SS, TMP, ' '))
//            {
//                String2Double.push_back(std::stod(TMP));
//            }
//
//            Point.x = String2Double[0];
//            Point.y = String2Double[1];
//            Point.z = String2Double[2];
//            Point.r = static_cast<int>(String2Double[3]);
//            Point.g = static_cast<int>(String2Double[4]);
//            Point.b = static_cast<int>(String2Double[5]);
//            Point.a = static_cast<int>(String2Double[String2Double.size() - 1]);
//            voCloud->push_back(Point);
//        }
//        Reader.close();
//    };
//
//	std::string PredictedCloudPath = "E:\\desktop\\Paper\\Exp\\binaryRF_vs_multiRF\\multiRF\\Scene1\\Interactions\\5\\Result.txt";
//	PointCloudT::Ptr pPredictedCloud=std::make_shared<PointCloudT>();
//    __loadPredictedCloud(PredictedCloudPath, pPredictedCloud);
//
//	std::string GroundTruthCloudPath = "E:\\desktop\\Paper\\Exp\\Comparsion of Online Method\\multiRF\\Scene1\\SubSample_mapd_Label_Randlanet_block_0_1.txt";
//	PointCloudT::Ptr pGroundTruthCloud = std::make_shared<PointCloudT>();
//    __loadGroundTruthCloud(GroundTruthCloudPath, pGroundTruthCloud);
//
//	computeAccByMapPredictedCloud2Groundtruth(pPredictedCloud, pGroundTruthCloud);
//}