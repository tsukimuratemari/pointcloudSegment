#include "pch.h"
#include "Utils.h"
#include "data.h"
#include "hyperparameters.h"
#include "online_mcboost.h"
#include "online_rf.h"
#include "experimenter.h"


//TEST(Comparsion_of_online_method, OMCBoost) {
//
//	std::string TESTDATADIR = "E:\\desktop\\Paper\\Exp\\Comparsion of Online Method\\OMCBoost\\Scene1\\";
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
//	Classifier* Model = new OnlineMCBoost(HP, TestDataset.m_numClasses, TestDataset.m_numFeatures, TestDataset.m_minFeatRange, TestDataset.m_maxFeatRange);
//	train(Model, TestDataset, HP);
//	vector<Result> Results = test(Model, TestDataset, HP);
//	PointCloudTool.visualizeORFApproachDLResult(InitialTrainData, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "FittingResult\\Approaching.txt", InitialTrainData);
//
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
//	////ManualResult approaching stage 6
//	//PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\6\\Positive.txt");
//	//NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\6\\Negative.txt");
//
//	//PointCloudTool.mergeMultiORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	//DataSet CorrectionSix(NumFeatures, FirstFeatureIndex, LabelIndex);
//	//CorrectionSix.load(ManualTrainData);
//	//train(Model, CorrectionSix, HP);
//	//Results.clear();
//	//Results = test(Model, TestDataset, HP);
//
//	//PointCloudTool.visualizeMultiORFApproachManualResult(InitialSegmentationResult, Results);
//	//PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\6\\Result.txt", InitialSegmentationResult);
//
//	////ManualResult approaching stage 7
//	//PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\7\\Positive.txt");
//	//NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\7\\Negative.txt");
//
//	//PointCloudTool.mergeMultiORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	//DataSet CorrectionSeven(NumFeatures, FirstFeatureIndex, LabelIndex);
//	//CorrectionSeven.load(ManualTrainData);
//	//train(Model, CorrectionSeven, HP);
//	//Results.clear();
//	//Results = test(Model, TestDataset, HP);
//
//	//PointCloudTool.visualizeMultiORFApproachManualResult(InitialSegmentationResult, Results);
//	//PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\7\\Result.txt", InitialSegmentationResult);
//
//	////ManualResult approaching stage 8
//	//PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\8\\Positive.txt");
//	//NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\8\\Negative.txt");
//
//	//PointCloudTool.mergeMultiORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	//DataSet CorrectionEight(NumFeatures, FirstFeatureIndex, LabelIndex);
//	//CorrectionEight.load(ManualTrainData);
//	//train(Model, CorrectionEight, HP);
//	//Results.clear();
//	//Results = test(Model, TestDataset, HP);
//
//	//PointCloudTool.visualizeMultiORFApproachManualResult(InitialSegmentationResult, Results);
//	//PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\8\\Result.txt", InitialSegmentationResult);
//}

TEST(Comparsion_of_online_method, DecisionTree) {

	std::string TESTDATADIR = "E:\\desktop\\Paper\\Exp\\Comparsion of Online Method\\DecisionTree\\Scene1\\";
	std::string InitialSegmentationResultPath = TESTDATADIR + "SubSample_mapd_Label_Randlanet_block_0_1.txt";
	std::string FeaturePath = TESTDATADIR + "Features\\AllFeatures.txt";
	std::string VehicleFeaturePath = TESTDATADIR + "Features\\VehicleFeatures.txt";
	std::string GroundFeaturePath = TESTDATADIR + "Features\\GroundFeatures.txt";
	std::string BuildingFeaturePath = TESTDATADIR + "Features\\BuildingFeatures.txt";
	std::string TreeFeaturePath = TESTDATADIR + "Features\\TreeFeatures.txt";

	std::string ConfFilePath = "..\\..\\OnlineLearning\\conf\\omcb.conf";
	Hyperparameters HP(ConfFilePath);
	CPointCloudTool PointCloudTool;
	std::vector<vector<double>> InitialSegmentationResult;
	int NumFeatures = 11, FirstFeatureIndex = 6, LabelIndex = 17;

	/************* Ground Correction ***************/
	PointCloudTool.loadOriginArray(FeaturePath, InitialSegmentationResult);
	PointCloudTool.deleteDuplicateConfidence(InitialSegmentationResult);
	auto InitialTrainData = InitialSegmentationResult;
	PointCloudTool.setLabelColumnIndex(LabelIndex);

	DataSet TestDataset;
	TestDataset.setNumFeatures(NumFeatures);
	TestDataset.setFirstFeatureColumnIndex(FirstFeatureIndex);
	TestDataset.setLaeblIndex(LabelIndex);
	TestDataset.load(InitialTrainData);
	Classifier* Model = new OnlineRF(HP, TestDataset.m_numClasses, TestDataset.m_numFeatures, TestDataset.m_minFeatRange, TestDataset.m_maxFeatRange);
	train(Model, TestDataset, HP);
	vector<Result> Results = test(Model, TestDataset, HP);
	PointCloudTool.visualizeORFApproachDLResult(InitialTrainData, Results);
	PointCloudTool.saveArrayFile(TESTDATADIR + "FittingResult\\Approaching.txt", InitialTrainData);

	//ManualResult approaching stage 1
	vector<string> PosManualResultFilePathVector;
	vector<string> NegManualResultFilePathVector;
	PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\1\\Positive.txt");
	NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\1\\Negative.txt");

	std::vector<vector<double>> ManualTrainData;
	PointCloudTool.mergeMultiORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
	DataSet CorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
	CorrectionOne.load(ManualTrainData);
	train(Model, CorrectionOne, HP);
	Results.clear();
	Results = test(Model, TestDataset, HP);

	PointCloudTool.visualizeMultiORFApproachManualResult(InitialSegmentationResult, Results);
	PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\1\\Result.txt", InitialSegmentationResult);

	//ManualResult approaching stage 2
	PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\2\\Positive.txt");
	NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\2\\Negative.txt");

	PointCloudTool.mergeMultiORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
	DataSet CorrectionTwo(NumFeatures, FirstFeatureIndex, LabelIndex);
	CorrectionTwo.load(ManualTrainData);
	train(Model, CorrectionTwo, HP);
	Results.clear();
	Results = test(Model, TestDataset, HP);

	PointCloudTool.visualizeMultiORFApproachManualResult(InitialSegmentationResult, Results);
	PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\2\\Result.txt", InitialSegmentationResult);

	//ManualResult approaching stage 3
	PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\3\\Positive.txt");
	NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\3\\Negative.txt");

	PointCloudTool.mergeMultiORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
	DataSet CorrectionThree(NumFeatures, FirstFeatureIndex, LabelIndex);
	CorrectionThree.load(ManualTrainData);
	train(Model, CorrectionThree, HP);
	Results.clear();
	Results = test(Model, TestDataset, HP);

	PointCloudTool.visualizeMultiORFApproachManualResult(InitialSegmentationResult, Results);
	PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\3\\Result.txt", InitialSegmentationResult);

	//ManualResult approaching stage 4
	PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\4\\Positive.txt");
	NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\4\\Negative.txt");

	PointCloudTool.mergeMultiORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
	DataSet CorrectionFour(NumFeatures, FirstFeatureIndex, LabelIndex);
	CorrectionFour.load(ManualTrainData);
	train(Model, CorrectionFour, HP);
	Results.clear();
	Results = test(Model, TestDataset, HP);

	PointCloudTool.visualizeMultiORFApproachManualResult(InitialSegmentationResult, Results);
	PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\4\\Result.txt", InitialSegmentationResult);

	//ManualResult approaching stage 5
	PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\5\\Positive.txt");
	NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\5\\Negative.txt");

	PointCloudTool.mergeMultiORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
	DataSet CorrectionFive(NumFeatures, FirstFeatureIndex, LabelIndex);
	CorrectionFive.load(ManualTrainData);
	train(Model, CorrectionFive, HP);
	Results.clear();
	Results = test(Model, TestDataset, HP);

	PointCloudTool.visualizeMultiORFApproachManualResult(InitialSegmentationResult, Results);
	PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\5\\Result.txt", InitialSegmentationResult);

	////ManualResult approaching stage 6
	//PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\6\\Positive.txt");
	//NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\6\\Negative.txt");

	//PointCloudTool.mergeMultiORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
	//DataSet CorrectionSix(NumFeatures, FirstFeatureIndex, LabelIndex);
	//CorrectionSix.load(ManualTrainData);
	//train(Model, CorrectionSix, HP);
	//Results.clear();
	//Results = test(Model, TestDataset, HP);

	//PointCloudTool.visualizeMultiORFApproachManualResult(InitialSegmentationResult, Results);
	//PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\6\\Result.txt", InitialSegmentationResult);

	////ManualResult approaching stage 7
	//PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\7\\Positive.txt");
	//NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\7\\Negative.txt");

	//PointCloudTool.mergeMultiORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
	//DataSet CorrectionSeven(NumFeatures, FirstFeatureIndex, LabelIndex);
	//CorrectionSeven.load(ManualTrainData);
	//train(Model, CorrectionSeven, HP);
	//Results.clear();
	//Results = test(Model, TestDataset, HP);

	//PointCloudTool.visualizeMultiORFApproachManualResult(InitialSegmentationResult, Results);
	//PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\7\\Result.txt", InitialSegmentationResult);

	////ManualResult approaching stage 8
	//PosManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\8\\Positive.txt");
	//NegManualResultFilePathVector.push_back(TESTDATADIR + "Interactions\\8\\Negative.txt");

	//PointCloudTool.mergeMultiORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
	//DataSet CorrectionEight(NumFeatures, FirstFeatureIndex, LabelIndex);
	//CorrectionEight.load(ManualTrainData);
	//train(Model, CorrectionEight, HP);
	//Results.clear();
	//Results = test(Model, TestDataset, HP);

	//PointCloudTool.visualizeMultiORFApproachManualResult(InitialSegmentationResult, Results);
	//PointCloudTool.saveArrayFile(TESTDATADIR + "Interactions\\8\\Result.txt", InitialSegmentationResult);
}