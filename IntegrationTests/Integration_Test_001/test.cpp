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
#include "SizeOfElevationDifferenceCluster.h"

class CTestIndependentFeature4EachSemantic :public testing::Test
{
public:
	std::string TESTDATADIR = "E:\\desktop\\Paper\\Exp\\resist_noise_level\\binaryclass\\0\\";
	std::string InitialSegmentationResultPath = TESTDATADIR + "Scene.txt";
	std::string FeaturePath = TESTDATADIR + "Features\\AllFeatures.txt";
	std::string VehicleFeaturePath = TESTDATADIR + "Features\\VehicleFeatures.txt";
	std::string GroundFeaturePath = TESTDATADIR + "Features\\GroundFeatures.txt";
	std::string BuildingFeaturePath = TESTDATADIR + "Features\\BuildingFeatures.txt";
	std::string TreeFeaturePath = TESTDATADIR + "Features\\TreeFeatures.txt";

protected:
	void SetUp() override
	{
		m_pCloud = std::make_shared<PointCloudT>();
		std::chrono::seconds Strat = std::chrono::seconds();
		loadCloud(InitialSegmentationResultPath, m_pCloud);
		_HIVE_SIMPLE_IF(!std::filesystem::exists(FeaturePath), _extractFeatures());
	}

	void TearDown() override
	{
		
	}

	void _extractFeatures()
	{
		auto Start = std::chrono::steady_clock::now();

		_HIVE_EARLY_EXIT(m_pCloud && m_pCloud->empty(), "point cloud should be initialized!");
		
		int Width = 128,Height=128;
		CProjectionElevation PEr(m_pCloud, Width, Height);
		std::unordered_map<int, float> ProjectElevations;
		PEr.compute(ProjectElevations);

		CElevationDifference EDr(m_pCloud);
		std::unordered_map<int, float> ElevationDifferences;
		EDr.compute(ElevationDifferences);
		std::unordered_map<int, float> ProjectElevationDifferences;
		EDr.computeBasedProjectElevation(ProjectElevationDifferences);

		CSizeOfElevationDifferenceCluster FeatureOperator(m_pCloud, ElevationDifferences);
		std::unordered_map<PointSerialNumber, FeatureValue> PointMapFeature;
		FeatureOperator.compute(PointMapFeature);

		CColorEntropy CEr(m_pCloud);
		std::unordered_map<int, float> ColorEntroys;
		float Radius = 0.7f;
		CEr.compute(ColorEntroys, Radius);

		float SmallRadius = 1.0f, LargeRadius = 5.0f;
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
			Features.emplace(i,SFeatures(
				ProjectElevations.at(i),
				ElevationDifferences.at(i),
				ProjectElevationDifferences.at(i),
				PointMapFeature.at(i),
				ColorEntroys.at(i),
				DoNs.at(i),
				LABs.at(i),
				CovarianceFeature.at(i)[5],
				ZDensityFeature.at(i)));
		}

		_writePositionAndFeatures(Features, FeaturePath);
		_writeVehicleFeatures(Features,VehicleFeaturePath);
		_writeGroundFeatures(Features, GroundFeaturePath);
		_writeBuildingFeatures(Features, BuildingFeaturePath);
		_writeTreeFeatures(Features, TreeFeaturePath);

		auto End = std::chrono::steady_clock::now();
		auto Duration = std::chrono::duration_cast<std::chrono::seconds>(End - Start);
		std::cout << "The timing cost for extract features is " << Duration.count() << "s" << std::endl;
	}

	void _writeTreeFeatures(const std::unordered_map<int, SFeatures>& vFeatures, const std::string& vPath)
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
			Writer << vFeatures.at(i)._Planarity << " ";
			Writer << vFeatures.at(i)._ZDensity << " ";
			Writer << vFeatures.at(i)._LAB[1] << " ";
			Writer << vFeatures.at(i)._LAB[2] << " ";
			Writer << (int)m_pCloud->points.at(i).a << " ";
			Writer << "\n";
		}
		Writer.close();
	}

	void _writeBuildingFeatures(const std::unordered_map<int, SFeatures>& vFeatures, const std::string& vPath)
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
			Writer << vFeatures.at(i)._DoN << " ";
			Writer << vFeatures.at(i)._Planarity << " ";
			Writer << vFeatures.at(i)._ZDensity << " ";
			Writer << (int)m_pCloud->points.at(i).a << " ";
			Writer << "\n";
		}
		Writer.close();
	}

	void _writeVehicleFeatures(const std::unordered_map<int, SFeatures>& vFeatures, const std::string& vPath)
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
			Writer << vFeatures.at(i)._SizeOfElevationDifferenceCluster << " ";
			Writer << (int)m_pCloud->points.at(i).a << " ";
			Writer << "\n";
		}
		Writer.close();
	}

	void _writeGroundFeatures(const std::unordered_map<int, SFeatures>& vFeatures, const std::string& vPath)
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
			Writer << vFeatures.at(i)._ZDensity << " ";
			Writer << (int)m_pCloud->points.at(i).a << " ";
			Writer << "\n";
		}
		Writer.close();

	}

	void _writePositionAndFeatures(const std::unordered_map<int,SFeatures>& vFeatures,const std::string& vPath)
	{
		std::ofstream Writer(vPath);
		Writer.setf(ios::fixed, ios::floatfield);
		Writer.precision(3);
		for (int i = 0;i < m_pCloud->size(); i++)
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
			Writer << vFeatures.at(i)._SizeOfElevationDifferenceCluster << " ";
			Writer << vFeatures.at(i)._DoN << " ";
			Writer << vFeatures.at(i)._ColorEntroy << " ";
			Writer << vFeatures.at(i)._LAB[0] << " ";
			Writer << vFeatures.at(i)._LAB[1] << " ";
			Writer << vFeatures.at(i)._LAB[2] << " ";
			Writer << vFeatures.at(i)._Planarity << " ";
			Writer << vFeatures.at(i)._ZDensity << " ";
			Writer << (int)m_pCloud->points.at(i).a << " ";
			Writer <<"\n";
		}
		Writer.close();
	}

private:
	PointCloudT::Ptr m_pCloud;
};

TEST_F(CTestIndependentFeature4EachSemantic, Run)
{
	;
}

//TEST_F(CTestIndependentFeature4EachSemantic, NT_NewScene1)
//{
//	std::string TESTDATADIR = "E:\\PointCloudDataSet\\scenes\\Stage2_Segmentation_NewScene\\Scene1\\";
//	std::string InitialSegmentationResultPath = TESTDATADIR + "SubSample-InitialSegmentationResult.txt";
//	std::string GroundTruth = TESTDATADIR + "GroundTruth.txt";
//	std::string FeaturePath = TESTDATADIR + "AllFeatures.txt";
//	std::string VehicleFeaturePath = TESTDATADIR + "VehicleFeatures.txt";
//	std::string GroundFeaturePath = TESTDATADIR + "GroundFeatures.txt";
//	std::string BuildingFeaturePath = TESTDATADIR + "BuildingFeatures.txt";
//	std::string TreeFeaturePath = TESTDATADIR + "TreeFeatures.txt";
//	std::string SubsequentDir = "E:\\PointCloudDataSet\\scenes\\SceneTest\\Scene5\\";
//	std::string ManualResultPath = "E:\\PointCloudDataSet\\scenes\\SceneTest\\Scene5\\";
//
//	std::string ConfFilePath = "..\\..\\OnlineLearning\\conf\\omcb.conf";
//	Hyperparameters HP(ConfFilePath);
//	CPointCloudTool PointCloudTool;
//	std::vector<vector<double>> InitialSegmentationResult;
//
//	/************* Building Correction ***************/
//	ESemanticCategory SemanticLabel = ESemanticCategory::Building;
//	PointCloudTool.loadOriginArray(BuildingFeaturePath, InitialSegmentationResult);
//	auto InitialTrainData4BuildingBinary = InitialSegmentationResult;
//	int LabelIndex = 8;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4BuildingBinary);
//	DataSet BuildingDataSet;
//	int NumFeatures = 2, FirstFeatureIndex = 6;
//	BuildingDataSet.setNumFeatures(NumFeatures);
//	BuildingDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
//	BuildingDataSet.setLaeblIndex(LabelIndex);
//	BuildingDataSet.load(InitialTrainData4BuildingBinary);
//	Classifier* Model = new OnlineRF(HP, BuildingDataSet.m_numClasses, BuildingDataSet.m_numFeatures, BuildingDataSet.m_minFeatRange, BuildingDataSet.m_maxFeatRange);
//
//	train(Model, BuildingDataSet, HP);
//	vector<Result> Results = test(Model, BuildingDataSet, HP);
//	PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4BuildingBinary, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "BuildingApproaching.txt", InitialTrainData4BuildingBinary);
//
//	//ManualResult approaching stage 1
//	vector<string> PosManualResultFilePathVector;
//	vector<string> NegManualResultFilePathVector;
//	PosManualResultFilePathVector.push_back(TESTDATADIR + "1\\Positive.txt");
//	NegManualResultFilePathVector.push_back(TESTDATADIR + "1\\Negative.txt");
//
//	std::vector<vector<double>> ManualTrainData;
//	PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	DataSet BuildingCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
//	BuildingCorrectionOne.load(ManualTrainData);
//	train(Model, BuildingCorrectionOne, HP);
//	Results.clear();
//	Results = test(Model, BuildingDataSet, HP);
//
//	int ConfidenceColumnIndex = 9;
//	PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "1\\Result.txt", InitialSegmentationResult);
//
//	//InitialSegmentationResult.clear();
//	//PointCloudTool.loadOriginArray(SubsequentDir + "BuildingFeatures.txt", InitialSegmentationResult);
//	//auto Test = InitialSegmentationResult;
//	//PointCloudTool.setLabelColumnIndex(LabelIndex);
//	//PointCloudTool.binarizeDLTrainData(SemanticLabel, Test);
//	//DataSet SubsequentBuildingDataSet(NumFeatures, FirstFeatureIndex, LabelIndex);
//	//SubsequentBuildingDataSet.load(Test);
//	//Results.clear();
//	//Results = test(Model, SubsequentBuildingDataSet, HP);
//	//PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	//PointCloudTool.saveArrayFile(ManualResultPath + "BuildingResult.txt", InitialSegmentationResult);
//
//	delete Model;
//
//
//	/************* Tree Correction ***************/
//	////Convergence to InitialSegmentation Result
//	SemanticLabel = ESemanticCategory::Tree;
//	InitialSegmentationResult.clear();
//	PointCloudTool.loadOriginArray(TreeFeaturePath, InitialSegmentationResult);
//
//	std::vector<std::vector<double>> LastCorrectionResult;
//	PointCloudTool.loadOriginArray(TESTDATADIR + "1\\Result.txt", LastCorrectionResult);
//	LabelIndex = 9;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastCorrectionResult);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "2\\ChangeResult.txt", InitialSegmentationResult);
//	auto InitialTrainData4TreeBinary = InitialSegmentationResult;
//
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4TreeBinary);
//	DataSet TreeDataSet;
//	NumFeatures = 3, FirstFeatureIndex = 6;
//	TreeDataSet.setNumFeatures(NumFeatures);
//	TreeDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
//	TreeDataSet.setLaeblIndex(LabelIndex);
//	TreeDataSet.load(InitialTrainData4TreeBinary);
//	Model = new OnlineRF(HP, 2, NumFeatures, TreeDataSet.m_minFeatRange, TreeDataSet.m_maxFeatRange);
//	train(Model, TreeDataSet, HP);
//	Results.clear();
//	Results = test(Model, TreeDataSet, HP);
//	PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4TreeBinary, Results);
//
//	//ManualResult approaching stage 1
//	PosManualResultFilePathVector.clear();
//	NegManualResultFilePathVector.clear();
//	PosManualResultFilePathVector.push_back(TESTDATADIR + "2\\Positive.txt");
//	NegManualResultFilePathVector.push_back(TESTDATADIR + "2\\Negative.txt");
//
//	ManualTrainData.clear();
//	PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	DataSet TreeCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
//	TreeCorrectionOne.load(ManualTrainData);
//	train(Model, TreeCorrectionOne, HP);
//	Results.clear();
//	Results = test(Model, TreeDataSet, HP);
//
//	ConfidenceColumnIndex = 10;
//	PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "2\\Result.txt", InitialSegmentationResult);
//
//	//InitialSegmentationResult.clear();
//	//PointCloudTool.loadOriginArray(SubsequentDir + "TreeFeatures.txt", InitialSegmentationResult);
//	//LastCorrectionResult.clear();
//	//PointCloudTool.loadOriginArray(ManualResultPath + "BuildingResult.txt", LastCorrectionResult);
//	//PointCloudTool.setLabelColumnIndex(LabelIndex);
//	//PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastCorrectionResult);
//	//InitialTrainData4TreeBinary.clear();
//	//InitialTrainData4TreeBinary = InitialSegmentationResult;
//	//PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4TreeBinary);
//	//DataSet TreeScene2DataSet(NumFeatures, FirstFeatureIndex, LabelIndex);
//	//TreeScene2DataSet.load(InitialTrainData4TreeBinary);
//	//Results.clear();
//	//Results = test(Model, TreeScene2DataSet, HP);
//	//PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	//PointCloudTool.saveArrayFile(ManualResultPath + "TreeResult.txt", InitialSegmentationResult);
//
//	delete Model;
//
//	/************* Ground Correction ***************/
//	////Convergence to InitialSegmentation Result
//	SemanticLabel = ESemanticCategory::Ground;
//	InitialSegmentationResult.clear();
//	PointCloudTool.loadOriginArray(GroundFeaturePath, InitialSegmentationResult);
//
//	LastCorrectionResult.clear();
//	PointCloudTool.loadOriginArray(TESTDATADIR + "2\\Result.txt", LastCorrectionResult);
//	LabelIndex = 9;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastCorrectionResult);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "3\\ChangeResult.txt", InitialSegmentationResult);
//	auto InitialTrainData4GroundBinary = InitialSegmentationResult;
//
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4GroundBinary);
//	DataSet GroundDataSet;
//	NumFeatures = 3, FirstFeatureIndex = 6;
//	GroundDataSet.setNumFeatures(NumFeatures);
//	GroundDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
//	GroundDataSet.setLaeblIndex(LabelIndex);
//	GroundDataSet.load(InitialTrainData4GroundBinary);
//	Model = new OnlineRF(HP, 2, NumFeatures, GroundDataSet.m_minFeatRange, GroundDataSet.m_maxFeatRange);
//	train(Model, GroundDataSet, HP);
//	Results.clear();
//	Results = test(Model, GroundDataSet, HP);
//	PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4GroundBinary, Results);
//	//PointCloudTool.saveArrayFile(GroundConvergencePath, InitialTrainData4GroundBinary);
//
//	//ManualResult approaching stage 1
//	PosManualResultFilePathVector.clear();
//	NegManualResultFilePathVector.clear();
//	PosManualResultFilePathVector.push_back(TESTDATADIR + "3\\Positive.txt");
//	NegManualResultFilePathVector.push_back(TESTDATADIR + "3\\Negative.txt");
//
//	ManualTrainData.clear();
//	PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	DataSet GroundCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
//	GroundCorrectionOne.load(ManualTrainData);
//	train(Model, GroundCorrectionOne, HP);
//	Results.clear();
//	Results = test(Model, GroundDataSet, HP);
//
//	ConfidenceColumnIndex = 10;
//	PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "3\\Result.txt", InitialSegmentationResult);
//
//
//	//InitialSegmentationResult.clear();
//	//PointCloudTool.loadOriginArray(SubsequentDir + "GroundFeatures.txt", InitialSegmentationResult);
//	//LastCorrectionResult.clear();
//	//PointCloudTool.loadOriginArray(ManualResultPath + "TreeResult.txt", LastCorrectionResult);
//	//PointCloudTool.setLabelColumnIndex(LabelIndex);
//	//PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastCorrectionResult);
//	//InitialTrainData4GroundBinary.clear();
//	//InitialTrainData4GroundBinary = InitialSegmentationResult;
//	//PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4GroundBinary);
//	//DataSet GroundScene2DataSet(NumFeatures, FirstFeatureIndex, LabelIndex);
//	//GroundScene2DataSet.load(InitialTrainData4GroundBinary);
//	//Results.clear();
//	//Results = test(Model, GroundScene2DataSet, HP);
//	//PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	//PointCloudTool.saveArrayFile(ManualResultPath + "GroundResult.txt", InitialSegmentationResult);
//
//	delete Model;
//
//}

//TEST_F(CTestIndependentFeature4EachSemantic, NT_NewScene2)
//{
//	std::string TESTDATADIR = "E:\\PointCloudDataSet\\scenes\\Stage2_Segmentation_NewScene\\Scene2\\";
//	std::string InitialSegmentationResultPath = TESTDATADIR + "SubSample-InitialSegmentationResult.txt";
//
//	std::string FeaturePath = TESTDATADIR + "AllFeatures.txt";
//	std::string VehicleFeaturePath = TESTDATADIR + "VehicleFeatures.txt";
//	std::string GroundFeaturePath = TESTDATADIR + "GroundFeatures.txt";
//	std::string BuildingFeaturePath = TESTDATADIR + "BuildingFeatures.txt";
//	std::string TreeFeaturePath = TESTDATADIR + "TreeFeatures.txt";
//	std::string SubsequentDir = "E:\\PointCloudDataSet\\scenes\\SceneTest\\Scene2\\";
//	std::string ManualResultPath = "E:\\PointCloudDataSet\\scenes\\SceneTest\\Scene2\\";
//	std::string ConfFilePath = "..\\..\\OnlineLearning\\conf\\omcb.conf";
//	Hyperparameters HP(ConfFilePath);
//	CPointCloudTool PointCloudTool;
//	std::vector<vector<double>> InitialSegmentationResult;
//
//	/************* Vehicle Correction ***************/
//	////Convergence to InitialSegmentation Result
//	ESemanticCategory SemanticLabel = ESemanticCategory::Vehicle;
//	PointCloudTool.loadOriginArray(VehicleFeaturePath, InitialSegmentationResult);
//	auto InitialTrainData4VehicleBinary = InitialSegmentationResult;
//	int LabelIndex = 10;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4VehicleBinary);
//	DataSet VehicleDataSet;
//	int NumFeatures = 4, FirstFeatureIndex = 6;
//	VehicleDataSet.setNumFeatures(NumFeatures);
//	VehicleDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
//	VehicleDataSet.setLaeblIndex(LabelIndex);
//	VehicleDataSet.load(InitialTrainData4VehicleBinary);
//	Classifier* Model = new OnlineRF(HP, VehicleDataSet.m_numClasses, VehicleDataSet.m_numFeatures, VehicleDataSet.m_minFeatRange, VehicleDataSet.m_maxFeatRange);
//	train(Model, VehicleDataSet, HP);
//	vector<Result> Results = test(Model, VehicleDataSet, HP);
//	PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4VehicleBinary, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "VehicileApproaching.txt", InitialTrainData4VehicleBinary);
//
//	//ManualResult approaching stage 1
//	vector<string> PosManualResultFilePathVector;
//	vector<string> NegManualResultFilePathVector;
//	PosManualResultFilePathVector.push_back(TESTDATADIR + "1\\Positive.txt");
//	NegManualResultFilePathVector.push_back(TESTDATADIR + "1\\Negative.txt");
//
//	std::vector<vector<double>> ManualTrainData;
//	PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	DataSet VehicleCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
//	VehicleCorrectionOne.load(ManualTrainData);
//	train(Model, VehicleCorrectionOne, HP);
//	Results.clear();
//	Results = test(Model, VehicleDataSet, HP);
//
//	int ConfidenceColumnIndex = 11;
//	PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "1\\Result.txt", InitialSegmentationResult);
//
//
//	InitialSegmentationResult.clear();
//	PointCloudTool.loadOriginArray(SubsequentDir + "VehicleFeatures.txt", InitialSegmentationResult);
//	auto Test = InitialSegmentationResult;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, Test);
//	DataSet SubsequentVehicleDataSet(NumFeatures, FirstFeatureIndex, LabelIndex);
//	SubsequentVehicleDataSet.load(Test);
//	Results.clear();
//	Results = test(Model, SubsequentVehicleDataSet, HP);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(ManualResultPath + "VehicleResult.txt", InitialSegmentationResult);
//
//	delete Model;
//
//	/************* Ground Correction ***************/
//	////Convergence to InitialSegmentation Result
//	SemanticLabel = ESemanticCategory::Ground;
//	InitialSegmentationResult.clear();
//	PointCloudTool.loadOriginArray(GroundFeaturePath, InitialSegmentationResult);
//
//	std::vector<std::vector<double>> LastCorrectionResult;
//	PointCloudTool.loadOriginArray(TESTDATADIR + "1\\Result.txt", LastCorrectionResult);
//	LabelIndex = 9;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastCorrectionResult);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "2\\ChangeResult.txt", InitialSegmentationResult);
//	auto InitialTrainData4GroundBinary = InitialSegmentationResult;
//
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4GroundBinary);
//	DataSet GroundDataSet;
//	NumFeatures = 3, FirstFeatureIndex = 6;
//	GroundDataSet.setNumFeatures(NumFeatures);
//	GroundDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
//	GroundDataSet.setLaeblIndex(LabelIndex);
//	GroundDataSet.load(InitialTrainData4GroundBinary);
//	Model = new OnlineRF(HP, GroundDataSet.m_numClasses, GroundDataSet.m_numFeatures, GroundDataSet.m_minFeatRange, GroundDataSet.m_maxFeatRange);
//	train(Model, GroundDataSet, HP);
//	Results.clear();
//	Results = test(Model, GroundDataSet, HP);
//	PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4GroundBinary, Results);
//
//	//ManualResult approaching stage 1
//	PosManualResultFilePathVector.clear();
//	NegManualResultFilePathVector.clear();
//	PosManualResultFilePathVector.push_back(TESTDATADIR + "2\\Positive.txt");
//	NegManualResultFilePathVector.push_back(TESTDATADIR + "2\\Negative.txt");
//
//	ManualTrainData.clear();
//	PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	DataSet GroundCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
//	GroundCorrectionOne.load(ManualTrainData);
//	train(Model, GroundCorrectionOne, HP);
//	Results.clear();
//	Results = test(Model, GroundDataSet, HP);
//
//	ConfidenceColumnIndex = 10;
//	PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "2\\Result.txt", InitialSegmentationResult);
//
//
//	InitialSegmentationResult.clear();
//	PointCloudTool.loadOriginArray(SubsequentDir + "GroundFeatures.txt", InitialSegmentationResult);
//	LastCorrectionResult.clear();
//	PointCloudTool.loadOriginArray(ManualResultPath + "VehicleResult.txt", LastCorrectionResult);
//	LabelIndex = 9;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastCorrectionResult);
//	InitialTrainData4GroundBinary.clear();
//	InitialTrainData4GroundBinary = InitialSegmentationResult;
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4GroundBinary);
//	DataSet GroundScene2DataSet(NumFeatures, FirstFeatureIndex, LabelIndex);
//	GroundScene2DataSet.load(InitialTrainData4GroundBinary);
//	Results.clear();
//	Results = test(Model, GroundScene2DataSet, HP);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(ManualResultPath + "GroundResult.txt", InitialSegmentationResult);
//
//	delete Model;
//}

//TEST_F(CTestIndependentFeature4EachSemantic, NT_NewScene3)
//{
//	std::string TESTDATADIR = "E:\\PointCloudDataSet\\scenes\\Stage2_Segmentation_NewScene\\Scene3\\";
//	std::string InitialSegmentationResultPath = TESTDATADIR + "SubSample-InitialSegmentationResult.txt";
//	std::string FeaturePath = TESTDATADIR + "AllFeatures.txt";
//	std::string VehicleFeaturePath = TESTDATADIR + "VehicleFeatures.txt";
//	std::string GroundFeaturePath = TESTDATADIR + "GroundFeatures.txt";
//	std::string BuildingFeaturePath = TESTDATADIR + "BuildingFeatures.txt";
//	std::string TreeFeaturePath = TESTDATADIR + "TreeFeatures.txt";
//	std::string SubsequentDir = "E:\\PointCloudDataSet\\scenes\\SceneTest\\Scene3\\";
//	std::string ManualResultPath = "E:\\PointCloudDataSet\\scenes\\SceneTest\\Scene3\\";
//
//	std::string ConfFilePath = "..\\..\\OnlineLearning\\conf\\omcb.conf";
//	Hyperparameters HP(ConfFilePath);
//	CPointCloudTool PointCloudTool;
//	std::vector<vector<double>> InitialSegmentationResult;
//
//	/************* Vehicle Correction ***************/
//	////Convergence to InitialSegmentation Result
//	ESemanticCategory SemanticLabel = ESemanticCategory::Vehicle;
//	PointCloudTool.loadOriginArray(VehicleFeaturePath, InitialSegmentationResult);
//	auto InitialTrainData4VehicleBinary = InitialSegmentationResult;
//	int LabelIndex = 10;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4VehicleBinary);
//	DataSet VehicleDataSet;
//	int NumFeatures = 4, FirstFeatureIndex = 6;
//	VehicleDataSet.setNumFeatures(NumFeatures);
//	VehicleDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
//	VehicleDataSet.setLaeblIndex(LabelIndex);
//	VehicleDataSet.load(InitialTrainData4VehicleBinary);
//	Classifier* Model = new OnlineRF(HP, VehicleDataSet.m_numClasses, VehicleDataSet.m_numFeatures, VehicleDataSet.m_minFeatRange, VehicleDataSet.m_maxFeatRange);
//	train(Model, VehicleDataSet, HP);
//	vector<Result> Results = test(Model, VehicleDataSet, HP);
//	PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4VehicleBinary, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "VehicileApproaching.txt", InitialTrainData4VehicleBinary);
//
//	//ManualResult approaching stage 1
//	vector<string> PosManualResultFilePathVector;
//	vector<string> NegManualResultFilePathVector;
//	PosManualResultFilePathVector.push_back(TESTDATADIR + "1\\Positive.txt");
//	NegManualResultFilePathVector.push_back(TESTDATADIR + "1\\Negative.txt");
//
//	std::vector<vector<double>> ManualTrainData;
//	PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	DataSet VehicleCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
//	VehicleCorrectionOne.load(ManualTrainData);
//	train(Model, VehicleCorrectionOne, HP);
//	Results.clear();
//	Results = test(Model, VehicleDataSet, HP);
//
//	int ConfidenceColumnIndex = 11;
//	PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "1\\Result.txt", InitialSegmentationResult);
//
//	//InitialSegmentationResult.clear();
//	//PointCloudTool.loadOriginArray(SubsequentDir + "VehicleFeatures.txt", InitialSegmentationResult);
//	//auto Test = InitialSegmentationResult;
//	//PointCloudTool.setLabelColumnIndex(LabelIndex);
//	//PointCloudTool.binarizeDLTrainData(SemanticLabel, Test);
//	//DataSet SubsequentVehicleDataSet(NumFeatures, FirstFeatureIndex, LabelIndex);
//	//SubsequentVehicleDataSet.load(Test);
//	//Results.clear();
//	//Results = test(Model, SubsequentVehicleDataSet, HP);
//	//PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	//PointCloudTool.saveArrayFile(ManualResultPath + "VehicleResult.txt", InitialSegmentationResult);
//
//	delete Model;
//
//	/************* Ground Correction ***************/
//	////Convergence to InitialSegmentation Result
//	SemanticLabel = ESemanticCategory::Ground;
//	InitialSegmentationResult.clear();
//	PointCloudTool.loadOriginArray(GroundFeaturePath, InitialSegmentationResult);
//
//	std::vector<std::vector<double>> LastCorrectionResult;
//	PointCloudTool.loadOriginArray(TESTDATADIR + "1\\Result.txt", LastCorrectionResult);
//	LabelIndex = 9;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastCorrectionResult);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "2\\ChangeResult.txt", InitialSegmentationResult);
//	auto InitialTrainData4GroundBinary = InitialSegmentationResult;
//
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4GroundBinary);
//	DataSet GroundDataSet;
//	NumFeatures = 3, FirstFeatureIndex = 6;
//	GroundDataSet.setNumFeatures(NumFeatures);
//	GroundDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
//	GroundDataSet.setLaeblIndex(LabelIndex);
//	GroundDataSet.load(InitialTrainData4GroundBinary);
//	Model = new OnlineRF(HP, GroundDataSet.m_numClasses, GroundDataSet.m_numFeatures, GroundDataSet.m_minFeatRange, GroundDataSet.m_maxFeatRange);
//	train(Model, GroundDataSet, HP);
//	Results.clear();
//	Results = test(Model, GroundDataSet, HP);
//	PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4GroundBinary, Results);
//	//PointCloudTool.saveArrayFile(GroundConvergencePath, InitialTrainData4GroundBinary);
//
//	//ManualResult approaching stage 1
//	PosManualResultFilePathVector.clear();
//	NegManualResultFilePathVector.clear();
//	PosManualResultFilePathVector.push_back(TESTDATADIR + "2\\Positive.txt");
//	NegManualResultFilePathVector.push_back(TESTDATADIR + "2\\Negative.txt");
//
//	ManualTrainData.clear();
//	PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	DataSet GroundCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
//	GroundCorrectionOne.load(ManualTrainData);
//	train(Model, GroundCorrectionOne, HP);
//	Results.clear();
//	Results = test(Model, GroundDataSet, HP);
//
//	ConfidenceColumnIndex = 10;
//	PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "2\\Result.txt", InitialSegmentationResult);
//
//
//	//InitialSegmentationResult.clear();
//	//PointCloudTool.loadOriginArray(SubsequentDir + "GroundFeatures.txt", InitialSegmentationResult);
//	//LastCorrectionResult.clear();
//	//PointCloudTool.loadOriginArray(ManualResultPath + "VehicleResult.txt", LastCorrectionResult);
//	//PointCloudTool.setLabelColumnIndex(LabelIndex);
//	//PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastCorrectionResult);
//	//InitialTrainData4GroundBinary.clear();
//	//InitialTrainData4GroundBinary = InitialSegmentationResult;
//	//PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4GroundBinary);
//	//DataSet GroundScene2DataSet(NumFeatures, FirstFeatureIndex, LabelIndex);
//	//GroundScene2DataSet.load(InitialTrainData4GroundBinary);
//	//Results.clear();
//	//Results = test(Model, GroundScene2DataSet, HP);
//	//PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	//PointCloudTool.saveArrayFile(ManualResultPath + "GroundResult.txt", InitialSegmentationResult);
//
//	delete Model;
//
//	/************* Tree Correction ***************/
//	////Convergence to InitialSegmentation Result
//	SemanticLabel = ESemanticCategory::Tree;
//	InitialSegmentationResult.clear();
//	PointCloudTool.loadOriginArray(TreeFeaturePath, InitialSegmentationResult);
//
//	LastCorrectionResult.clear();
//	PointCloudTool.loadOriginArray(TESTDATADIR + "2\\Result.txt", LastCorrectionResult);
//	LabelIndex = 9;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastCorrectionResult);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "3\\ChangeResult.txt", InitialSegmentationResult);
//	auto InitialTrainData4TreeBinary = InitialSegmentationResult;
//
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4TreeBinary);
//	DataSet TreeDataSet;
//	NumFeatures = 3, FirstFeatureIndex = 6;
//	TreeDataSet.setNumFeatures(NumFeatures);
//	TreeDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
//	TreeDataSet.setLaeblIndex(LabelIndex);
//	TreeDataSet.load(InitialTrainData4TreeBinary);
//	Model = new OnlineRF(HP, TreeDataSet.m_numClasses, TreeDataSet.m_numFeatures, TreeDataSet.m_minFeatRange, TreeDataSet.m_maxFeatRange);
//	train(Model, TreeDataSet, HP);
//	Results.clear();
//	Results = test(Model, TreeDataSet, HP);
//	PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4TreeBinary, Results);
//
//	//ManualResult approaching stage 1
//	PosManualResultFilePathVector.clear();
//	NegManualResultFilePathVector.clear();
//	PosManualResultFilePathVector.push_back(TESTDATADIR + "3\\Positive.txt");
//	NegManualResultFilePathVector.push_back(TESTDATADIR + "3\\Negative.txt");
//
//	ManualTrainData.clear();
//	PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	DataSet TreeCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
//	TreeCorrectionOne.load(ManualTrainData);
//	train(Model, TreeCorrectionOne, HP);
//	Results.clear();
//	Results = test(Model, TreeDataSet, HP);
//
//	ConfidenceColumnIndex = 10;
//	PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "3\\Result.txt", InitialSegmentationResult);
//
//
//	//InitialSegmentationResult.clear();
//	//PointCloudTool.loadOriginArray(SubsequentDir + "TreeFeatures.txt", InitialSegmentationResult);
//	//LastCorrectionResult.clear();
//	//PointCloudTool.loadOriginArray(ManualResultPath + "GroundResult.txt", LastCorrectionResult);
//	//PointCloudTool.setLabelColumnIndex(LabelIndex);
//	//PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastCorrectionResult);
//	//InitialTrainData4TreeBinary.clear();
//	//InitialTrainData4TreeBinary = InitialSegmentationResult;
//	//PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4TreeBinary);
//	//DataSet TreeScene2DataSet(NumFeatures, FirstFeatureIndex, LabelIndex);
//	//TreeScene2DataSet.load(InitialTrainData4TreeBinary);
//	//Results.clear();
//	//Results = test(Model, TreeScene2DataSet, HP);
//	//PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	//PointCloudTool.saveArrayFile(ManualResultPath + "TreeResult.txt", InitialSegmentationResult);
//
//	delete Model;
//}

//TEST_F(CTestIndependentFeature4EachSemantic, NT_NewScene4)
//{
//	std::string TESTDATADIR = "E:\\PointCloudDataSet\\scenes\\Stage2_Segmentation_NewScene\\Scene4\\";
//	std::string InitialSegmentationResultPath = TESTDATADIR + "SubSample-InitialSegmentationResult.txt";
//	std::string FeaturePath = TESTDATADIR + "AllFeatures.txt";
//	std::string VehicleFeaturePath = TESTDATADIR + "VehicleFeatures.txt";
//	std::string GroundFeaturePath = TESTDATADIR + "GroundFeatures.txt";
//	std::string BuildingFeaturePath = TESTDATADIR + "BuildingFeatures.txt";
//	std::string TreeFeaturePath = TESTDATADIR + "TreeFeatures.txt";
//	std::string SubsequentDir = "E:\\PointCloudDataSet\\scenes\\SceneTest\\Scene3\\";
//	std::string ManualResultPath = "E:\\PointCloudDataSet\\scenes\\SceneTest\\Scene3\\";
//	std::string ConfFilePath = "..\\..\\OnlineLearning\\conf\\omcb.conf";
//	Hyperparameters HP(ConfFilePath);
//	CPointCloudTool PointCloudTool;
//	std::vector<vector<double>> InitialSegmentationResult;
//
//	/************* Ground Correction ***************/
//	//Convergence to InitialSegmentation Result
//	ESemanticCategory SemanticLabel = ESemanticCategory::Ground;
//	PointCloudTool.loadOriginArray(GroundFeaturePath, InitialSegmentationResult);
//	auto InitialTrainData4GroundBinary = InitialSegmentationResult;
//	int LabelIndex = 9;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4GroundBinary);
//	DataSet GroundDataSet;
//	int NumFeatures = 3, FirstFeatureIndex = 6;
//	GroundDataSet.setNumFeatures(NumFeatures);
//	GroundDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
//	GroundDataSet.setLaeblIndex(LabelIndex);
//	GroundDataSet.load(InitialTrainData4GroundBinary);
//	Classifier* Model = new OnlineRF(HP, GroundDataSet.m_numClasses, GroundDataSet.m_numFeatures, GroundDataSet.m_minFeatRange, GroundDataSet.m_maxFeatRange);
//	train(Model, GroundDataSet, HP);
//	vector<Result> Results = test(Model, GroundDataSet, HP);
//	PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4GroundBinary, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "GroundApproaching.txt", InitialTrainData4GroundBinary);
//
//	//ManualResult approaching stage 1
//	vector<string> PosManualResultFilePathVector;
//	vector<string> NegManualResultFilePathVector;
//	PosManualResultFilePathVector.push_back(TESTDATADIR + "1\\Positive.txt");
//	NegManualResultFilePathVector.push_back(TESTDATADIR + "1\\Negative.txt");
//
//	std::vector<vector<double>> ManualTrainData;
//	PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	DataSet GroundCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
//	GroundCorrectionOne.load(ManualTrainData);
//	train(Model, GroundCorrectionOne, HP);
//	Results.clear();
//	Results = test(Model, GroundDataSet, HP);
//
//	int ConfidenceColumnIndex = 10;
//	PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "1\\Result.txt", InitialSegmentationResult);
//
//	//InitialSegmentationResult.clear();
//	//PointCloudTool.loadOriginArray(SubsequentDir + "VehicleFeatures.txt", InitialSegmentationResult);
//	//auto Test = InitialSegmentationResult;
//	//PointCloudTool.setLabelColumnIndex(LabelIndex);
//	//PointCloudTool.binarizeDLTrainData(SemanticLabel, Test);
//	//DataSet SubsequentVehicleDataSet(NumFeatures, FirstFeatureIndex, LabelIndex);
//	//SubsequentVehicleDataSet.load(Test);
//	//Results.clear();
//	//Results = test(Model, SubsequentVehicleDataSet, HP);
//	//PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	//PointCloudTool.saveArrayFile(ManualResultPath + "VehicleResult.txt", InitialSegmentationResult);
//
//	delete Model;
//
//	/************* Tree Correction ***************/
//	////Convergence to InitialSegmentation Result
//	SemanticLabel = ESemanticCategory::Tree;
//	InitialSegmentationResult.clear();
//	PointCloudTool.loadOriginArray(TreeFeaturePath, InitialSegmentationResult);
//
//	std::vector<std::vector<double>> LastCorrectionResult;
//	PointCloudTool.loadOriginArray(TESTDATADIR + "1\\Result.txt", LastCorrectionResult);
//	LabelIndex = 9;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastCorrectionResult);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "2\\ChangeResult.txt", InitialSegmentationResult);
//	auto InitialTrainData4TreeBinary = InitialSegmentationResult;
//
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4TreeBinary);
//	DataSet TreeDataSet;
//	NumFeatures = 3, FirstFeatureIndex = 6;
//	TreeDataSet.setNumFeatures(NumFeatures);
//	TreeDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
//	TreeDataSet.setLaeblIndex(LabelIndex);
//	TreeDataSet.load(InitialTrainData4TreeBinary);
//	Model = new OnlineRF(HP, TreeDataSet.m_numClasses, TreeDataSet.m_numFeatures, TreeDataSet.m_minFeatRange, TreeDataSet.m_maxFeatRange);
//	train(Model, TreeDataSet, HP);
//	Results.clear();
//	Results = test(Model, TreeDataSet, HP);
//	PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4TreeBinary, Results);
//
//	//ManualResult approaching stage 1
//	PosManualResultFilePathVector.clear();
//	NegManualResultFilePathVector.clear();
//	PosManualResultFilePathVector.push_back(TESTDATADIR + "2\\Positive.txt");
//	NegManualResultFilePathVector.push_back(TESTDATADIR + "2\\Negative.txt");
//
//	ManualTrainData.clear();
//	PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	DataSet TreeCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
//	TreeCorrectionOne.load(ManualTrainData);
//	train(Model, TreeCorrectionOne, HP);
//	Results.clear();
//	Results = test(Model, TreeDataSet, HP);
//
//	ConfidenceColumnIndex = 10;
//	PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "2\\Result.txt", InitialSegmentationResult);
//
//
//	//InitialSegmentationResult.clear();
//	//PointCloudTool.loadOriginArray(SubsequentDir + "TreeFeatures.txt", InitialSegmentationResult);
//	//LastCorrectionResult.clear();
//	//PointCloudTool.loadOriginArray(ManualResultPath + "GroundResult.txt", LastCorrectionResult);
//	//PointCloudTool.setLabelColumnIndex(LabelIndex);
//	//PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastCorrectionResult);
//	//InitialTrainData4TreeBinary.clear();
//	//InitialTrainData4TreeBinary = InitialSegmentationResult;
//	//PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4TreeBinary);
//	//DataSet TreeScene2DataSet(NumFeatures, FirstFeatureIndex, LabelIndex);
//	//TreeScene2DataSet.load(InitialTrainData4TreeBinary);
//	//Results.clear();
//	//Results = test(Model, TreeScene2DataSet, HP);
//	//PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	//PointCloudTool.saveArrayFile(ManualResultPath + "TreeResult.txt", InitialSegmentationResult);
//
//	delete Model;
//}

//TEST_F(CTestIndependentFeature4EachSemantic, NT_NewScene5)
//{
//	std::string TESTDATADIR = "E:\\PointCloudDataSet\\scenes\\Stage2_Segmentation_NewScene\\Scene5\\";
//	std::string InitialSegmentationResultPath = TESTDATADIR + "SubSample-InitialSegmentationResult.txt";
//	std::string GroundTruth = TESTDATADIR + "GroundTruth.txt";
//	std::string FeaturePath = TESTDATADIR + "AllFeatures.txt";
//	std::string VehicleFeaturePath = TESTDATADIR + "VehicleFeatures.txt";
//	std::string GroundFeaturePath = TESTDATADIR + "GroundFeatures.txt";
//	std::string BuildingFeaturePath = TESTDATADIR + "BuildingFeatures.txt";
//	std::string TreeFeaturePath = TESTDATADIR + "TreeFeatures.txt";
//	std::string SubsequentDir = "E:\\PointCloudDataSet\\scenes\\SceneTest\\Scene5\\";
//	std::string ManualResultPath = "E:\\PointCloudDataSet\\scenes\\SceneTest\\Scene5\\";
//
//	std::string ConfFilePath = "..\\..\\OnlineLearning\\conf\\omcb.conf";
//	Hyperparameters HP(ConfFilePath);
//	CPointCloudTool PointCloudTool;
//	std::vector<vector<double>> InitialSegmentationResult;
//
//	/************* Building Correction ***************/
//	ESemanticCategory SemanticLabel = ESemanticCategory::Building;
//	PointCloudTool.loadOriginArray(BuildingFeaturePath, InitialSegmentationResult);
//	auto InitialTrainData4BuildingBinary = InitialSegmentationResult;
//	int LabelIndex = 8;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4BuildingBinary);
//	DataSet BuildingDataSet;
//	int NumFeatures = 2, FirstFeatureIndex = 6;
//	BuildingDataSet.setNumFeatures(NumFeatures);
//	BuildingDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
//	BuildingDataSet.setLaeblIndex(LabelIndex);
//	BuildingDataSet.load(InitialTrainData4BuildingBinary);
//	Classifier* Model = new OnlineRF(HP, BuildingDataSet.m_numClasses, BuildingDataSet.m_numFeatures, BuildingDataSet.m_minFeatRange, BuildingDataSet.m_maxFeatRange);
//
//	train(Model, BuildingDataSet, HP);
//	vector<Result> Results = test(Model, BuildingDataSet, HP);
//	PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4BuildingBinary, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "BuildingApproaching.txt", InitialTrainData4BuildingBinary);
//
//	//ManualResult approaching stage 1
//	vector<string> PosManualResultFilePathVector;
//	vector<string> NegManualResultFilePathVector;
//	PosManualResultFilePathVector.push_back(TESTDATADIR + "1\\Positive.txt");
//	NegManualResultFilePathVector.push_back(TESTDATADIR + "1\\Negative.txt");
//
//	std::vector<vector<double>> ManualTrainData;
//	PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	DataSet BuildingCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
//	BuildingCorrectionOne.load(ManualTrainData);
//	train(Model, BuildingCorrectionOne, HP);
//	Results.clear();
//	Results = test(Model, BuildingDataSet, HP);
//
//	int ConfidenceColumnIndex = 9;
//	PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "1\\Result.txt", InitialSegmentationResult);
//
//	//InitialSegmentationResult.clear();
//	//PointCloudTool.loadOriginArray(SubsequentDir + "BuildingFeatures.txt", InitialSegmentationResult);
//	//auto Test = InitialSegmentationResult;
//	//PointCloudTool.setLabelColumnIndex(LabelIndex);
//	//PointCloudTool.binarizeDLTrainData(SemanticLabel, Test);
//	//DataSet SubsequentBuildingDataSet(NumFeatures, FirstFeatureIndex, LabelIndex);
//	//SubsequentBuildingDataSet.load(Test);
//	//Results.clear();
//	//Results = test(Model, SubsequentBuildingDataSet, HP);
//	//PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	//PointCloudTool.saveArrayFile(ManualResultPath + "BuildingResult.txt", InitialSegmentationResult);
//
//	delete Model;
//
//
//	/************* Tree Correction ***************/
//	////Convergence to InitialSegmentation Result
//	SemanticLabel = ESemanticCategory::Tree;
//	InitialSegmentationResult.clear();
//	PointCloudTool.loadOriginArray(TreeFeaturePath, InitialSegmentationResult);
//
//	std::vector<std::vector<double>> LastCorrectionResult;
//	PointCloudTool.loadOriginArray(TESTDATADIR + "1\\Result.txt", LastCorrectionResult);
//	LabelIndex = 9;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastCorrectionResult);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "2\\ChangeResult.txt", InitialSegmentationResult);
//	auto InitialTrainData4TreeBinary = InitialSegmentationResult;
//
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4TreeBinary);
//	DataSet TreeDataSet;
//	NumFeatures = 3, FirstFeatureIndex = 6;
//	TreeDataSet.setNumFeatures(NumFeatures);
//	TreeDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
//	TreeDataSet.setLaeblIndex(LabelIndex);
//	TreeDataSet.load(InitialTrainData4TreeBinary);
//	Model = new OnlineRF(HP, 2, NumFeatures, TreeDataSet.m_minFeatRange, TreeDataSet.m_maxFeatRange);
//	train(Model, TreeDataSet, HP);
//	Results.clear();
//	Results = test(Model, TreeDataSet, HP);
//	PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4TreeBinary, Results);
//
//	//ManualResult approaching stage 1
//	PosManualResultFilePathVector.clear();
//	NegManualResultFilePathVector.clear();
//	PosManualResultFilePathVector.push_back(TESTDATADIR + "2\\Positive.txt");
//	NegManualResultFilePathVector.push_back(TESTDATADIR + "2\\Negative.txt");
//
//	ManualTrainData.clear();
//	PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	DataSet TreeCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
//	TreeCorrectionOne.load(ManualTrainData);
//	train(Model, TreeCorrectionOne, HP);
//	Results.clear();
//	Results = test(Model, TreeDataSet, HP);
//
//	ConfidenceColumnIndex = 10;
//	PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "2\\Result.txt", InitialSegmentationResult);
//
//	//InitialSegmentationResult.clear();
//	//PointCloudTool.loadOriginArray(SubsequentDir + "TreeFeatures.txt", InitialSegmentationResult);
//	//LastCorrectionResult.clear();
//	//PointCloudTool.loadOriginArray(ManualResultPath + "BuildingResult.txt", LastCorrectionResult);
//	//PointCloudTool.setLabelColumnIndex(LabelIndex);
//	//PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastCorrectionResult);
//	//InitialTrainData4TreeBinary.clear();
//	//InitialTrainData4TreeBinary = InitialSegmentationResult;
//	//PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4TreeBinary);
//	//DataSet TreeScene2DataSet(NumFeatures, FirstFeatureIndex, LabelIndex);
//	//TreeScene2DataSet.load(InitialTrainData4TreeBinary);
//	//Results.clear();
//	//Results = test(Model, TreeScene2DataSet, HP);
//	//PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	//PointCloudTool.saveArrayFile(ManualResultPath + "TreeResult.txt", InitialSegmentationResult);
//
//	delete Model;
//
//	/************* Ground Correction ***************/
//	////Convergence to InitialSegmentation Result
//	SemanticLabel = ESemanticCategory::Ground;
//	InitialSegmentationResult.clear();
//	PointCloudTool.loadOriginArray(GroundFeaturePath, InitialSegmentationResult);
//
//	LastCorrectionResult.clear();
//	PointCloudTool.loadOriginArray(TESTDATADIR + "2\\Result.txt", LastCorrectionResult);
//	LabelIndex = 9;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastCorrectionResult);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "3\\ChangeResult.txt", InitialSegmentationResult);
//	auto InitialTrainData4GroundBinary = InitialSegmentationResult;
//
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4GroundBinary);
//	DataSet GroundDataSet;
//	NumFeatures = 3, FirstFeatureIndex = 6;
//	GroundDataSet.setNumFeatures(NumFeatures);
//	GroundDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
//	GroundDataSet.setLaeblIndex(LabelIndex);
//	GroundDataSet.load(InitialTrainData4GroundBinary);
//	Model = new OnlineRF(HP, 2, NumFeatures, GroundDataSet.m_minFeatRange, GroundDataSet.m_maxFeatRange);
//	train(Model, GroundDataSet, HP);
//	Results.clear();
//	Results = test(Model, GroundDataSet, HP);
//	PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4GroundBinary, Results);
//	//PointCloudTool.saveArrayFile(GroundConvergencePath, InitialTrainData4GroundBinary);
//
//	//ManualResult approaching stage 1
//	PosManualResultFilePathVector.clear();
//	NegManualResultFilePathVector.clear();
//	PosManualResultFilePathVector.push_back(TESTDATADIR + "3\\Positive.txt");
//	NegManualResultFilePathVector.push_back(TESTDATADIR + "3\\Negative.txt");
//
//	ManualTrainData.clear();
//	PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	DataSet GroundCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
//	GroundCorrectionOne.load(ManualTrainData);
//	train(Model, GroundCorrectionOne, HP);
//	Results.clear();
//	Results = test(Model, GroundDataSet, HP);
//
//	ConfidenceColumnIndex = 10;
//	PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "3\\Result.txt", InitialSegmentationResult);
//
//
//	//InitialSegmentationResult.clear();
//	//PointCloudTool.loadOriginArray(SubsequentDir + "GroundFeatures.txt", InitialSegmentationResult);
//	//LastCorrectionResult.clear();
//	//PointCloudTool.loadOriginArray(ManualResultPath + "TreeResult.txt", LastCorrectionResult);
//	//PointCloudTool.setLabelColumnIndex(LabelIndex);
//	//PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastCorrectionResult);
//	//InitialTrainData4GroundBinary.clear();
//	//InitialTrainData4GroundBinary = InitialSegmentationResult;
//	//PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4GroundBinary);
//	//DataSet GroundScene2DataSet(NumFeatures, FirstFeatureIndex, LabelIndex);
//	//GroundScene2DataSet.load(InitialTrainData4GroundBinary);
//	//Results.clear();
//	//Results = test(Model, GroundScene2DataSet, HP);
//	//PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	//PointCloudTool.saveArrayFile(ManualResultPath + "GroundResult.txt", InitialSegmentationResult);
//
//	delete Model;
//
//}

//TEST_F(CTestIndependentFeature4EachSemantic, NT_NewScene6)
//{
//	std::string TESTDATADIR = "E:\\PointCloudDataSet\\scenes\\Stage2_Segmentation_NewScene\\Scene6\\";
//	std::string InitialSegmentationResultPath = TESTDATADIR + "SubSample-InitialSegmentationResult.txt";
//	std::string FeaturePath = TESTDATADIR + "AllFeatures.txt";
//	std::string VehicleFeaturePath = TESTDATADIR + "VehicleFeatures.txt";
//	std::string GroundFeaturePath = TESTDATADIR + "GroundFeatures.txt";
//	std::string BuildingFeaturePath = TESTDATADIR + "BuildingFeatures.txt";
//	std::string TreeFeaturePath = TESTDATADIR + "TreeFeatures.txt";
//	std::string SubsequentDir = "E:\\PointCloudDataSet\\scenes\\SceneTest\\Scene3\\";
//	std::string ManualResultPath = "E:\\PointCloudDataSet\\scenes\\SceneTest\\Scene3\\";
//
//	std::string ConfFilePath = "..\\..\\OnlineLearning\\conf\\omcb.conf";
//	Hyperparameters HP(ConfFilePath);
//	CPointCloudTool PointCloudTool;
//	std::vector<vector<double>> InitialSegmentationResult;
//
//	/************* Vehicle Correction ***************/
//	////Convergence to InitialSegmentation Result
//	ESemanticCategory SemanticLabel = ESemanticCategory::Vehicle;
//	PointCloudTool.loadOriginArray(VehicleFeaturePath, InitialSegmentationResult);
//	auto InitialTrainData4VehicleBinary = InitialSegmentationResult;
//	int LabelIndex = 10;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4VehicleBinary);
//	DataSet VehicleDataSet;
//	int NumFeatures = 4, FirstFeatureIndex = 6;
//	VehicleDataSet.setNumFeatures(NumFeatures);
//	VehicleDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
//	VehicleDataSet.setLaeblIndex(LabelIndex);
//	VehicleDataSet.load(InitialTrainData4VehicleBinary);
//	Classifier* Model = new OnlineRF(HP, VehicleDataSet.m_numClasses, VehicleDataSet.m_numFeatures, VehicleDataSet.m_minFeatRange, VehicleDataSet.m_maxFeatRange);
//	train(Model, VehicleDataSet, HP);
//	vector<Result> Results = test(Model, VehicleDataSet, HP);
//	PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4VehicleBinary, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "VehicileApproaching.txt", InitialTrainData4VehicleBinary);
//
//	//ManualResult approaching stage 1
//	vector<string> PosManualResultFilePathVector;
//	vector<string> NegManualResultFilePathVector;
//	PosManualResultFilePathVector.push_back(TESTDATADIR + "1\\Positive.txt");
//	NegManualResultFilePathVector.push_back(TESTDATADIR + "1\\Negative.txt");
//
//	std::vector<vector<double>> ManualTrainData;
//	PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	DataSet VehicleCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
//	VehicleCorrectionOne.load(ManualTrainData);
//	train(Model, VehicleCorrectionOne, HP);
//	Results.clear();
//	Results = test(Model, VehicleDataSet, HP);
//
//	int ConfidenceColumnIndex = 11;
//	PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "1\\Result.txt", InitialSegmentationResult);
//
//	//InitialSegmentationResult.clear();
//	//PointCloudTool.loadOriginArray(SubsequentDir + "VehicleFeatures.txt", InitialSegmentationResult);
//	//auto Test = InitialSegmentationResult;
//	//PointCloudTool.setLabelColumnIndex(LabelIndex);
//	//PointCloudTool.binarizeDLTrainData(SemanticLabel, Test);
//	//DataSet SubsequentVehicleDataSet(NumFeatures, FirstFeatureIndex, LabelIndex);
//	//SubsequentVehicleDataSet.load(Test);
//	//Results.clear();
//	//Results = test(Model, SubsequentVehicleDataSet, HP);
//	//PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	//PointCloudTool.saveArrayFile(ManualResultPath + "VehicleResult.txt", InitialSegmentationResult);
//
//	delete Model;
//
//	/************* Ground Correction ***************/
//	////Convergence to InitialSegmentation Result
//	SemanticLabel = ESemanticCategory::Ground;
//	InitialSegmentationResult.clear();
//	PointCloudTool.loadOriginArray(GroundFeaturePath, InitialSegmentationResult);
//
//	std::vector<std::vector<double>> LastCorrectionResult;
//	PointCloudTool.loadOriginArray(TESTDATADIR + "1\\Result.txt", LastCorrectionResult);
//	LabelIndex = 9;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastCorrectionResult);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "2\\ChangeResult.txt", InitialSegmentationResult);
//	auto InitialTrainData4GroundBinary = InitialSegmentationResult;
//
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4GroundBinary);
//	DataSet GroundDataSet;
//	NumFeatures = 3, FirstFeatureIndex = 6;
//	GroundDataSet.setNumFeatures(NumFeatures);
//	GroundDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
//	GroundDataSet.setLaeblIndex(LabelIndex);
//	GroundDataSet.load(InitialTrainData4GroundBinary);
//	Model = new OnlineRF(HP, GroundDataSet.m_numClasses, GroundDataSet.m_numFeatures, GroundDataSet.m_minFeatRange, GroundDataSet.m_maxFeatRange);
//	train(Model, GroundDataSet, HP);
//	Results.clear();
//	Results = test(Model, GroundDataSet, HP);
//	PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4GroundBinary, Results);
//	//PointCloudTool.saveArrayFile(GroundConvergencePath, InitialTrainData4GroundBinary);
//
//	//ManualResult approaching stage 1
//	PosManualResultFilePathVector.clear();
//	NegManualResultFilePathVector.clear();
//	PosManualResultFilePathVector.push_back(TESTDATADIR + "2\\Positive.txt");
//	NegManualResultFilePathVector.push_back(TESTDATADIR + "2\\Negative.txt");
//
//	ManualTrainData.clear();
//	PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	DataSet GroundCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
//	GroundCorrectionOne.load(ManualTrainData);
//	train(Model, GroundCorrectionOne, HP);
//	Results.clear();
//	Results = test(Model, GroundDataSet, HP);
//
//	ConfidenceColumnIndex = 10;
//	PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "2\\Result.txt", InitialSegmentationResult);
//
//
//	//InitialSegmentationResult.clear();
//	//PointCloudTool.loadOriginArray(SubsequentDir + "GroundFeatures.txt", InitialSegmentationResult);
//	//LastCorrectionResult.clear();
//	//PointCloudTool.loadOriginArray(ManualResultPath + "VehicleResult.txt", LastCorrectionResult);
//	//PointCloudTool.setLabelColumnIndex(LabelIndex);
//	//PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastCorrectionResult);
//	//InitialTrainData4GroundBinary.clear();
//	//InitialTrainData4GroundBinary = InitialSegmentationResult;
//	//PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4GroundBinary);
//	//DataSet GroundScene2DataSet(NumFeatures, FirstFeatureIndex, LabelIndex);
//	//GroundScene2DataSet.load(InitialTrainData4GroundBinary);
//	//Results.clear();
//	//Results = test(Model, GroundScene2DataSet, HP);
//	//PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	//PointCloudTool.saveArrayFile(ManualResultPath + "GroundResult.txt", InitialSegmentationResult);
//
//	delete Model;
//
//	/************* Tree Correction ***************/
//	////Convergence to InitialSegmentation Result
//	SemanticLabel = ESemanticCategory::Tree;
//	InitialSegmentationResult.clear();
//	PointCloudTool.loadOriginArray(TreeFeaturePath, InitialSegmentationResult);
//
//	LastCorrectionResult.clear();
//	PointCloudTool.loadOriginArray(TESTDATADIR + "2\\Result.txt", LastCorrectionResult);
//	LabelIndex = 9;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastCorrectionResult);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "3\\ChangeResult.txt", InitialSegmentationResult);
//	auto InitialTrainData4TreeBinary = InitialSegmentationResult;
//
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4TreeBinary);
//	DataSet TreeDataSet;
//	NumFeatures = 3, FirstFeatureIndex = 6;
//	TreeDataSet.setNumFeatures(NumFeatures);
//	TreeDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
//	TreeDataSet.setLaeblIndex(LabelIndex);
//	TreeDataSet.load(InitialTrainData4TreeBinary);
//	Model = new OnlineRF(HP, TreeDataSet.m_numClasses, TreeDataSet.m_numFeatures, TreeDataSet.m_minFeatRange, TreeDataSet.m_maxFeatRange);
//	train(Model, TreeDataSet, HP);
//	Results.clear();
//	Results = test(Model, TreeDataSet, HP);
//	PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4TreeBinary, Results);
//
//	//ManualResult approaching stage 1
//	PosManualResultFilePathVector.clear();
//	NegManualResultFilePathVector.clear();
//	PosManualResultFilePathVector.push_back(TESTDATADIR + "3\\Positive.txt");
//	NegManualResultFilePathVector.push_back(TESTDATADIR + "3\\Negative.txt");
//
//	ManualTrainData.clear();
//	PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	DataSet TreeCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
//	TreeCorrectionOne.load(ManualTrainData);
//	train(Model, TreeCorrectionOne, HP);
//	Results.clear();
//	Results = test(Model, TreeDataSet, HP);
//
//	ConfidenceColumnIndex = 10;
//	PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "3\\Result.txt", InitialSegmentationResult);
//
//
//	//InitialSegmentationResult.clear();
//	//PointCloudTool.loadOriginArray(SubsequentDir + "TreeFeatures.txt", InitialSegmentationResult);
//	//LastCorrectionResult.clear();
//	//PointCloudTool.loadOriginArray(ManualResultPath + "GroundResult.txt", LastCorrectionResult);
//	//PointCloudTool.setLabelColumnIndex(LabelIndex);
//	//PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastCorrectionResult);
//	//InitialTrainData4TreeBinary.clear();
//	//InitialTrainData4TreeBinary = InitialSegmentationResult;
//	//PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4TreeBinary);
//	//DataSet TreeScene2DataSet(NumFeatures, FirstFeatureIndex, LabelIndex);
//	//TreeScene2DataSet.load(InitialTrainData4TreeBinary);
//	//Results.clear();
//	//Results = test(Model, TreeScene2DataSet, HP);
//	//PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	//PointCloudTool.saveArrayFile(ManualResultPath + "TreeResult.txt", InitialSegmentationResult);
//
//	delete Model;
//}
//
///*****************************************************/
//
//TEST_F(CTestIndependentFeature4EachSemantic, NT_Scene5)
//{
//	std::string TESTDATADIR = "E:\\PointCloudDataSet\\scenes\\Scene4\\";
//	std::string InitialSegmentationResultPath = TESTDATADIR + "SubSample-InitialSegmentationResult.txt";
//	std::string GroundTruth = TESTDATADIR + "GroundTruth.txt";
//	std::string FeaturePath = TESTDATADIR + "AllFeatures.txt";
//	std::string VehicleFeaturePath = TESTDATADIR + "VehicleFeatures.txt";
//	std::string GroundFeaturePath = TESTDATADIR + "GroundFeatures.txt";
//	std::string BuildingFeaturePath = TESTDATADIR + "BuildingFeatures.txt";
//	std::string TreeFeaturePath = TESTDATADIR + "TreeFeatures.txt";
//	std::string SubsequentDir = "E:\\PointCloudDataSet\\scenes\\SceneTest\\Scene1\\";
//	std::string ManualResultPath = "E:\\PointCloudDataSet\\scenes\\SceneTest\\Scene1\\";
//	std::string ConfFilePath = "..\\..\\OnlineLearning\\conf\\omcb.conf";
//	Hyperparameters HP(ConfFilePath);
//	CPointCloudTool PointCloudTool;
//	std::vector<vector<double>> InitialSegmentationResult;
//
//	/************* Vehicle Correction ***************/
//	////Convergence to InitialSegmentation Result
//	ESemanticCategory SemanticLabel = ESemanticCategory::Vehicle;
//	PointCloudTool.loadOriginArray(VehicleFeaturePath, InitialSegmentationResult);
//	auto InitialTrainData4VehicleBinary = InitialSegmentationResult;
//	int LabelIndex = 10;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4VehicleBinary);
//	DataSet VehicleDataSet;
//	int NumFeatures = 4, FirstFeatureIndex = 6;
//	VehicleDataSet.setNumFeatures(NumFeatures);
//	VehicleDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
//	VehicleDataSet.setLaeblIndex(LabelIndex);
//	VehicleDataSet.load(InitialTrainData4VehicleBinary);
//	Classifier* Model = new OnlineRF(HP, VehicleDataSet.m_numClasses, VehicleDataSet.m_numFeatures, VehicleDataSet.m_minFeatRange, VehicleDataSet.m_maxFeatRange);
//	train(Model, VehicleDataSet, HP);
//	vector<Result> Results = test(Model, VehicleDataSet, HP);
//	PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4VehicleBinary, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "VehicileApproaching.txt", InitialTrainData4VehicleBinary);
//
//	//ManualResult approaching stage 1
//	vector<string> PosManualResultFilePathVector;
//	vector<string> NegManualResultFilePathVector;
//	PosManualResultFilePathVector.push_back(TESTDATADIR + "1\\Positive.txt");
//	NegManualResultFilePathVector.push_back(TESTDATADIR + "1\\Negative.txt");
//
//	std::vector<vector<double>> ManualTrainData;
//	PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	DataSet VehicleCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
//	VehicleCorrectionOne.load(ManualTrainData);
//	train(Model, VehicleCorrectionOne, HP);
//	Results.clear();
//	Results = test(Model, VehicleDataSet, HP);
//
//	int ConfidenceColumnIndex = 11;
//	PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "1\\Result.txt", InitialSegmentationResult);
//
//
//	InitialSegmentationResult.clear();
//	PointCloudTool.loadOriginArray(SubsequentDir + "VehicleFeatures.txt", InitialSegmentationResult);
//	auto Test = InitialSegmentationResult;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, Test);
//	DataSet SubsequentVehicleDataSet(NumFeatures, FirstFeatureIndex, LabelIndex);
//	SubsequentVehicleDataSet.load(Test);
//	Results.clear();
//	Results = test(Model, SubsequentVehicleDataSet, HP);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(ManualResultPath + "VehicleResult.txt", InitialSegmentationResult);
//
//	delete Model;
//
//	/************* Ground Correction ***************/
//	////Convergence to InitialSegmentation Result
//	SemanticLabel = ESemanticCategory::Ground;
//	InitialSegmentationResult.clear();
//	PointCloudTool.loadOriginArray(GroundFeaturePath, InitialSegmentationResult);
//
//	std::vector<std::vector<double>> LastCorrectionResult;
//	PointCloudTool.loadOriginArray(TESTDATADIR + "1\\Result.txt", LastCorrectionResult);
//	LabelIndex = 9;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastCorrectionResult);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "2\\ChangeResult.txt", InitialSegmentationResult);
//	auto InitialTrainData4GroundBinary = InitialSegmentationResult;
//
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4GroundBinary);
//	DataSet GroundDataSet;
//	NumFeatures = 3, FirstFeatureIndex = 6;
//	GroundDataSet.setNumFeatures(NumFeatures);
//	GroundDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
//	GroundDataSet.setLaeblIndex(LabelIndex);
//	GroundDataSet.load(InitialTrainData4GroundBinary);
//	Model = new OnlineRF(HP, GroundDataSet.m_numClasses, GroundDataSet.m_numFeatures, GroundDataSet.m_minFeatRange, GroundDataSet.m_maxFeatRange);
//	train(Model, GroundDataSet, HP);
//	Results.clear();
//	Results = test(Model, GroundDataSet, HP);
//	PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4GroundBinary, Results);
//
//	//ManualResult approaching stage 1
//	PosManualResultFilePathVector.clear();
//	NegManualResultFilePathVector.clear();
//	PosManualResultFilePathVector.push_back(TESTDATADIR + "2\\Positive.txt");
//	NegManualResultFilePathVector.push_back(TESTDATADIR + "2\\Negative.txt");
//
//	ManualTrainData.clear();
//	PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	DataSet GroundCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
//	GroundCorrectionOne.load(ManualTrainData);
//	train(Model, GroundCorrectionOne, HP);
//	Results.clear();
//	Results = test(Model, GroundDataSet, HP);
//
//	ConfidenceColumnIndex = 10;
//	PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "2\\Result.txt", InitialSegmentationResult);
//
//	InitialSegmentationResult.clear();
//	PointCloudTool.loadOriginArray(SubsequentDir + "GroundFeatures.txt", InitialSegmentationResult);
//	LastCorrectionResult.clear();
//	PointCloudTool.loadOriginArray(ManualResultPath + "VehicleResult.txt", LastCorrectionResult);
//	LabelIndex = 9;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastCorrectionResult);
//	InitialTrainData4GroundBinary.clear();
//	InitialTrainData4GroundBinary = InitialSegmentationResult;
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4GroundBinary);
//	DataSet GroundScene2DataSet(NumFeatures, FirstFeatureIndex, LabelIndex);
//	GroundScene2DataSet.load(InitialTrainData4GroundBinary);
//	Results.clear();
//	Results = test(Model, GroundScene2DataSet, HP);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(ManualResultPath + "GroundResult.txt", InitialSegmentationResult);
//
//	delete Model;
//
//	/************* Tree Correction ***************/
//	////Convergence to InitialSegmentation Result
//	SemanticLabel = ESemanticCategory::Tree;
//	InitialSegmentationResult.clear();
//	PointCloudTool.loadOriginArray(TreeFeaturePath, InitialSegmentationResult);
//
//	LastCorrectionResult.clear();
//	PointCloudTool.loadOriginArray(TESTDATADIR + "2\\Result.txt", LastCorrectionResult);
//	LabelIndex = 9;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastCorrectionResult);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "3\\ChangeResult.txt", InitialSegmentationResult);
//	auto InitialTrainData4TreeBinary = InitialSegmentationResult;
//
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4TreeBinary);
//	DataSet TreeDataSet;
//	NumFeatures = 3, FirstFeatureIndex = 6;
//	TreeDataSet.setNumFeatures(NumFeatures);
//	TreeDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
//	TreeDataSet.setLaeblIndex(LabelIndex);
//	TreeDataSet.load(InitialTrainData4TreeBinary);
//	Model = new OnlineRF(HP, TreeDataSet.m_numClasses, TreeDataSet.m_numFeatures, TreeDataSet.m_minFeatRange, TreeDataSet.m_maxFeatRange);
//	train(Model, TreeDataSet, HP);
//	Results.clear();
//	Results = test(Model, TreeDataSet, HP);
//	PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4TreeBinary, Results);
//
//	//ManualResult approaching stage 1
//	PosManualResultFilePathVector.clear();
//	NegManualResultFilePathVector.clear();
//	PosManualResultFilePathVector.push_back(TESTDATADIR + "3\\Positive.txt");
//	NegManualResultFilePathVector.push_back(TESTDATADIR + "3\\Negative.txt");
//
//	ManualTrainData.clear();
//	PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	DataSet TreeCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
//	TreeCorrectionOne.load(ManualTrainData);
//	train(Model, TreeCorrectionOne, HP);
//	Results.clear();
//	Results = test(Model, TreeDataSet, HP);
//
//	ConfidenceColumnIndex = 10;
//	PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "3\\Result.txt", InitialSegmentationResult);
//
//
//	InitialSegmentationResult.clear();
//	PointCloudTool.loadOriginArray(SubsequentDir + "TreeFeatures.txt", InitialSegmentationResult);
//	LastCorrectionResult.clear();
//	PointCloudTool.loadOriginArray(ManualResultPath + "GroundResult.txt", LastCorrectionResult);
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastCorrectionResult);
//	InitialTrainData4TreeBinary.clear();
//	InitialTrainData4TreeBinary = InitialSegmentationResult;
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4TreeBinary);
//	DataSet TreeScene2DataSet(NumFeatures, FirstFeatureIndex, LabelIndex);
//	TreeScene2DataSet.load(InitialTrainData4TreeBinary);
//	Results.clear();
//	Results = test(Model, TreeScene2DataSet, HP);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(ManualResultPath + "TreeResult.txt", InitialSegmentationResult);
//
//
//	delete Model;
//
//
//	/************* Building Correction ***************/
//	////Convergence to InitialSegmentation Result
//	SemanticLabel = ESemanticCategory::Building;
//	InitialSegmentationResult.clear();
//	PointCloudTool.loadOriginArray(BuildingFeaturePath, InitialSegmentationResult);
//
//	//std::vector<std::vector<double>> LastCorrectionResult;
//	LastCorrectionResult.clear();
//	PointCloudTool.loadOriginArray(TESTDATADIR + "3\\Result.txt", LastCorrectionResult);
//	LabelIndex = 8;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastCorrectionResult);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "4\\ChangeResult.txt", InitialSegmentationResult);
//	auto InitialTrainData4BuildingBinary = InitialSegmentationResult;
//
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4BuildingBinary);
//	DataSet BuildingDataSet;
//	NumFeatures = 2, FirstFeatureIndex = 6;
//	BuildingDataSet.setNumFeatures(NumFeatures);
//	BuildingDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
//	BuildingDataSet.setLaeblIndex(LabelIndex);
//	BuildingDataSet.load(InitialTrainData4BuildingBinary);
//	Model = new OnlineRF(HP, 2, NumFeatures, BuildingDataSet.m_minFeatRange, BuildingDataSet.m_maxFeatRange);
//	train(Model, BuildingDataSet, HP);
//	Results.clear();
//	Results = test(Model, BuildingDataSet, HP);
//	PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4BuildingBinary, Results);
//	//PointCloudTool.saveArrayFile(GroundConvergencePath, InitialTrainData4TreeBinary);
//
//	//ManualResult approaching stage 1
//	PosManualResultFilePathVector.clear();
//	NegManualResultFilePathVector.clear();
//	PosManualResultFilePathVector.push_back(TESTDATADIR + "4\\Positive.txt");
//	NegManualResultFilePathVector.push_back(TESTDATADIR + "4\\Negative.txt");
//
//	ManualTrainData.clear();
//	PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	DataSet BuildingCollectOne(NumFeatures, FirstFeatureIndex, LabelIndex);
//	BuildingCollectOne.load(ManualTrainData);
//	train(Model, BuildingCollectOne, HP);
//	vector<Result> BuildingResults;
//	BuildingResults = test(Model, BuildingDataSet, HP);
//
//	ConfidenceColumnIndex = 9;
//	PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, BuildingResults);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "4\\Result.txt", InitialSegmentationResult);
//
//	InitialSegmentationResult.clear();
//	PointCloudTool.loadOriginArray(SubsequentDir + "BuildingFeatures.txt", InitialSegmentationResult);
//	LastCorrectionResult.clear();
//	PointCloudTool.loadOriginArray(ManualResultPath + "TreeResult.txt", LastCorrectionResult);
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastCorrectionResult);
//	InitialTrainData4BuildingBinary.clear();
//	InitialTrainData4BuildingBinary = InitialSegmentationResult;
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4BuildingBinary);
//	DataSet BuildingScene2DataSet(NumFeatures, FirstFeatureIndex, LabelIndex);
//	BuildingScene2DataSet.load(InitialTrainData4BuildingBinary);
//	BuildingResults.clear();
//	BuildingResults = test(Model, BuildingScene2DataSet, HP);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, BuildingResults);
//	PointCloudTool.saveArrayFile(ManualResultPath + "BuildingResult.txt", InitialSegmentationResult);
//
//	delete Model;
//
//}
//
//TEST_F(CTestIndependentFeature4EachSemantic, NT_Scene1)
//{
//	std::string TESTDATADIR = "E:\\PointCloudDataSet\\scenes\\Scene0\\";
//	std::string InitialSegmentationResultPath = TESTDATADIR + "SubSample-InitialSegmentationResult.txt";
//	std::string GroundTruth = TESTDATADIR + "GroundTruth.txt";
//	std::string FeaturePath = TESTDATADIR + "AllFeatures.txt";
//	std::string VehicleFeaturePath = TESTDATADIR + "VehicleFeatures.txt";
//	std::string GroundFeaturePath = TESTDATADIR + "GroundFeatures.txt";
//	std::string BuildingFeaturePath = TESTDATADIR + "BuildingFeatures.txt";
//	std::string TreeFeaturePath = TESTDATADIR + "TreeFeatures.txt";
//	std::string SubsequentDir = "E:\\PointCloudDataSet\\scenes\\SceneTest\\Scene1\\";
//	std::string ManualResultPath = "E:\\PointCloudDataSet\\scenes\\SceneTest\\Scene1\\";
//
//	std::string ConfFilePath = "..\\..\\OnlineLearning\\conf\\omcb.conf";
//	Hyperparameters HP(ConfFilePath);
//	CPointCloudTool PointCloudTool;
//	std::vector<vector<double>> InitialSegmentationResult;
//	/************* Vehicle Correction ***************/
//	//Convergence to InitialSegmentation Result
//	ESemanticCategory SemanticLabel = ESemanticCategory::Vehicle;
//	PointCloudTool.loadOriginArray(VehicleFeaturePath, InitialSegmentationResult);
//	auto InitialTrainData4VehicleBinary = InitialSegmentationResult;
//	int LabelIndex = 10;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4VehicleBinary);
//	DataSet VehicleDataSet;
//	int NumFeatures = 4, FirstFeatureIndex = 6;
//	VehicleDataSet.setNumFeatures(NumFeatures);
//	VehicleDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
//	VehicleDataSet.setLaeblIndex(LabelIndex);
//	VehicleDataSet.load(InitialTrainData4VehicleBinary);
//	Classifier* Model = new OnlineRF(HP, VehicleDataSet.m_numClasses, VehicleDataSet.m_numFeatures, VehicleDataSet.m_minFeatRange, VehicleDataSet.m_maxFeatRange);
//	train(Model, VehicleDataSet, HP);
//	vector<Result> Results = test(Model, VehicleDataSet, HP);
//	PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4VehicleBinary, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "VehicileApproaching.txt", InitialTrainData4VehicleBinary);
//
//	//ManualResult approaching stage 1
//	vector<string> PosManualResultFilePathVector;
//	vector<string> NegManualResultFilePathVector;
//	PosManualResultFilePathVector.push_back(TESTDATADIR + "1\\Positive.txt");
//	NegManualResultFilePathVector.push_back(TESTDATADIR + "1\\Negative.txt");
//
//	std::vector<vector<double>> ManualTrainData;
//	PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	DataSet VehicleCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
//	VehicleCorrectionOne.load(ManualTrainData);
//	train(Model, VehicleCorrectionOne, HP);
//	Results.clear();
//	Results = test(Model, VehicleDataSet, HP);
//
//	int ConfidenceColumnIndex = 11;
//	PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "1\\Result.txt", InitialSegmentationResult);
//
//	//ManualResult approaching stage 2
//	PosManualResultFilePathVector.clear();
//	NegManualResultFilePathVector.clear();
//	PosManualResultFilePathVector.push_back(TESTDATADIR + "2\\Positive.txt");
//	NegManualResultFilePathVector.push_back(TESTDATADIR + "2\\Negative.txt");
//
//	ManualTrainData.clear();
//	PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	DataSet VehicleCorrectionTwo(NumFeatures, FirstFeatureIndex, LabelIndex);
//	VehicleCorrectionTwo.load(ManualTrainData);
//	train(Model, VehicleCorrectionTwo, HP);
//	Results.clear();
//	Results = test(Model, VehicleDataSet, HP);
//
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "2\\Result.txt", InitialSegmentationResult);
//
//	InitialSegmentationResult.clear();
//	PointCloudTool.loadOriginArray(SubsequentDir + "VehicleFeatures.txt", InitialSegmentationResult);
//	auto Test = InitialSegmentationResult;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, Test);
//	DataSet SubsequentVehicleDataSet(NumFeatures, FirstFeatureIndex, LabelIndex);
//	SubsequentVehicleDataSet.load(Test);
//	Results.clear();
//	Results = test(Model, SubsequentVehicleDataSet, HP);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(ManualResultPath + "VehicleResult.txt", InitialSegmentationResult);
//
//	delete Model;
//
//	/************* Ground Correction ***************/
//	//Convergence to InitialSegmentation Result
//	SemanticLabel = ESemanticCategory::Ground;
//	InitialSegmentationResult.clear();
//	PointCloudTool.loadOriginArray(GroundFeaturePath, InitialSegmentationResult);
//
//	std::vector<std::vector<double>> LastCorrectionResult;
//	PointCloudTool.loadOriginArray(TESTDATADIR + "2\\Result.txt", LastCorrectionResult);
//	LabelIndex = 9;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastCorrectionResult);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "3\\ChangeResult.txt", InitialSegmentationResult);
//	auto InitialTrainData4GroundBinary = InitialSegmentationResult;
//
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4GroundBinary);
//	DataSet GroundDataSet;
//	NumFeatures = 3, FirstFeatureIndex = 6;
//	GroundDataSet.setNumFeatures(NumFeatures);
//	GroundDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
//	GroundDataSet.setLaeblIndex(LabelIndex);
//	GroundDataSet.load(InitialTrainData4GroundBinary);
//	Model = new OnlineRF(HP, GroundDataSet.m_numClasses, GroundDataSet.m_numFeatures, GroundDataSet.m_minFeatRange, GroundDataSet.m_maxFeatRange);
//	train(Model, GroundDataSet, HP);
//	Results.clear();
//	Results = test(Model, GroundDataSet, HP);
//	PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4GroundBinary, Results);
//	//PointCloudTool.saveArrayFile(GroundConvergencePath, InitialTrainData4GroundBinary);
//
//	//ManualResult approaching stage 1
//	PosManualResultFilePathVector.clear();
//	NegManualResultFilePathVector.clear();
//	PosManualResultFilePathVector.push_back(TESTDATADIR + "3\\Positive.txt");
//	NegManualResultFilePathVector.push_back(TESTDATADIR + "3\\Negative.txt");
//
//	ManualTrainData.clear();
//	PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	DataSet GroundCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
//	GroundCorrectionOne.load(ManualTrainData);
//	train(Model, GroundCorrectionOne, HP);
//	Results.clear();
//	Results = test(Model, GroundDataSet, HP);
//
//	ConfidenceColumnIndex = 10;
//	PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "3\\Result.txt", InitialSegmentationResult);
//
//	//ManualResult approaching stage 2
//	PosManualResultFilePathVector.clear();
//	NegManualResultFilePathVector.clear();
//	PosManualResultFilePathVector.push_back(TESTDATADIR + "4\\Positive.txt");
//	NegManualResultFilePathVector.push_back(TESTDATADIR + "4\\Negative.txt");
//
//	ManualTrainData.clear();
//	PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	DataSet GroundCorrectionTwo(NumFeatures, FirstFeatureIndex, LabelIndex);
//	GroundCorrectionTwo.load(ManualTrainData);
//	train(Model, GroundCorrectionTwo, HP);
//	Results.clear();
//	Results = test(Model, GroundDataSet, HP);
//
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "4\\Result.txt", InitialSegmentationResult);
//
//	InitialSegmentationResult.clear();
//	PointCloudTool.loadOriginArray(SubsequentDir + "GroundFeatures.txt", InitialSegmentationResult);
//	LastCorrectionResult.clear();
//	PointCloudTool.loadOriginArray(ManualResultPath + "VehicleResult.txt", LastCorrectionResult);
//	LabelIndex = 9;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastCorrectionResult);
//	InitialTrainData4GroundBinary.clear();
//	InitialTrainData4GroundBinary = InitialSegmentationResult;
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4GroundBinary);
//	DataSet GroundScene2DataSet(NumFeatures, FirstFeatureIndex, LabelIndex);
//	GroundScene2DataSet.load(InitialTrainData4GroundBinary);
//	Results.clear();
//	Results = test(Model, GroundScene2DataSet, HP);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(ManualResultPath + "GroundResult.txt", InitialSegmentationResult);
//
//	delete Model;
//
//	/************* Tree Correction ***************/
//	//Convergence to InitialSegmentation Result
//	SemanticLabel = ESemanticCategory::Tree;
//	InitialSegmentationResult.clear();
//	PointCloudTool.loadOriginArray(TreeFeaturePath, InitialSegmentationResult);
//
//
//	LastCorrectionResult.clear();
//	PointCloudTool.loadOriginArray(TESTDATADIR + "4\\Result.txt", LastCorrectionResult);
//	LabelIndex = 9;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastCorrectionResult);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "5\\ChangeResult.txt", InitialSegmentationResult);
//	auto InitialTrainData4TreeBinary = InitialSegmentationResult;
//
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4TreeBinary);
//	DataSet TreeDataSet;
//	NumFeatures = 3, FirstFeatureIndex = 6;
//	TreeDataSet.setNumFeatures(NumFeatures);
//	TreeDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
//	TreeDataSet.setLaeblIndex(LabelIndex);
//	TreeDataSet.load(InitialTrainData4TreeBinary);
//	Model = new OnlineRF(HP, TreeDataSet.m_numClasses, TreeDataSet.m_numFeatures, TreeDataSet.m_minFeatRange, TreeDataSet.m_maxFeatRange);
//	train(Model, TreeDataSet, HP);
//	Results.clear();
//	Results = test(Model, TreeDataSet, HP);
//	PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4TreeBinary, Results);
//	//PointCloudTool.saveArrayFile(GroundConvergencePath, InitialTrainData4TreeBinary);
//
//	//ManualResult approaching stage 1
//	PosManualResultFilePathVector.clear();
//	NegManualResultFilePathVector.clear();
//	PosManualResultFilePathVector.push_back(TESTDATADIR + "5\\Positive.txt");
//	NegManualResultFilePathVector.push_back(TESTDATADIR + "5\\Negative.txt");
//
//	ManualTrainData.clear();
//	PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	DataSet TreeCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
//	TreeCorrectionOne.load(ManualTrainData);
//	train(Model, TreeCorrectionOne, HP);
//	Results.clear();
//	Results = test(Model, TreeDataSet, HP);
//
//	ConfidenceColumnIndex = 10;
//	PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "5\\Result.txt", InitialSegmentationResult);
//
//	InitialSegmentationResult.clear();
//	PointCloudTool.loadOriginArray(SubsequentDir + "TreeFeatures.txt", InitialSegmentationResult);
//	LastCorrectionResult.clear();
//	PointCloudTool.loadOriginArray(ManualResultPath + "GroundResult.txt", LastCorrectionResult);
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastCorrectionResult);
//	InitialTrainData4TreeBinary.clear();
//	InitialTrainData4TreeBinary = InitialSegmentationResult;
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4TreeBinary);
//	DataSet TreeScene2DataSet(NumFeatures, FirstFeatureIndex, LabelIndex);
//	TreeScene2DataSet.load(InitialTrainData4TreeBinary);
//	Results.clear();
//	Results = test(Model, TreeScene2DataSet, HP);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(ManualResultPath + "TreeResult.txt", InitialSegmentationResult);
//
//	delete Model;
//
//
//	/************* Building Correction ***************/
//	//Convergence to InitialSegmentation Result
//	SemanticLabel = ESemanticCategory::Building;
//	InitialSegmentationResult.clear();
//	PointCloudTool.loadOriginArray(BuildingFeaturePath, InitialSegmentationResult);
//
//
//	LastCorrectionResult.clear();
//	PointCloudTool.loadOriginArray(TESTDATADIR + "5\\Result.txt", LastCorrectionResult);
//	LabelIndex = 8;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastCorrectionResult);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "6\\ChangeResult.txt", InitialSegmentationResult);
//	auto InitialTrainData4BuildingBinary = InitialSegmentationResult;
//
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4BuildingBinary);
//	DataSet BuildingDataSet;
//	NumFeatures = 2, FirstFeatureIndex = 6;
//	BuildingDataSet.setNumFeatures(NumFeatures);
//	BuildingDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
//	BuildingDataSet.setLaeblIndex(LabelIndex);
//	BuildingDataSet.load(InitialTrainData4BuildingBinary);
//	Model = new OnlineRF(HP, BuildingDataSet.m_numClasses, BuildingDataSet.m_numFeatures, BuildingDataSet.m_minFeatRange, BuildingDataSet.m_maxFeatRange);
//	train(Model, BuildingDataSet, HP);
//	Results.clear();
//	Results = test(Model, BuildingDataSet, HP);
//	PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4BuildingBinary, Results);
//	//PointCloudTool.saveArrayFile(GroundConvergencePath, InitialTrainData4TreeBinary);
//
//	//ManualResult approaching stage 1
//	PosManualResultFilePathVector.clear();
//	NegManualResultFilePathVector.clear();
//	PosManualResultFilePathVector.push_back(TESTDATADIR + "6\\Positive.txt");
//	NegManualResultFilePathVector.push_back(TESTDATADIR + "6\\Negative.txt");
//
//	ManualTrainData.clear();
//	PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	DataSet BuildingCollectOne(NumFeatures, FirstFeatureIndex, LabelIndex);
//	BuildingCollectOne.load(ManualTrainData);
//	train(Model, BuildingCollectOne, HP);
//	Results.clear();
//	Results = test(Model, BuildingDataSet, HP);
//
//	ConfidenceColumnIndex = 9;
//	PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "6\\Result.txt", InitialSegmentationResult);
//
//	InitialSegmentationResult.clear();
//	PointCloudTool.loadOriginArray(SubsequentDir + "BuildingFeatures.txt", InitialSegmentationResult);
//	LastCorrectionResult.clear();
//	PointCloudTool.loadOriginArray(ManualResultPath + "TreeResult.txt", LastCorrectionResult);
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastCorrectionResult);
//	InitialTrainData4BuildingBinary.clear();
//	InitialTrainData4BuildingBinary = InitialSegmentationResult;
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4TreeBinary);
//	DataSet BuildingScene2DataSet(NumFeatures, FirstFeatureIndex, LabelIndex);
//	BuildingScene2DataSet.load(InitialTrainData4BuildingBinary);
//	Results.clear();
//	Results = test(Model, BuildingScene2DataSet, HP);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(ManualResultPath + "BuildingResult.txt", InitialSegmentationResult);
//
//	delete Model;
//
//}
//
//TEST_F(CTestIndependentFeature4EachSemantic, NT_Scene4)
//{
//
//	std::string TESTDATADIR = "E:\\PointCloudDataSet\\scenes\\Scene3\\";
//	std::string InitialSegmentationResultPath = TESTDATADIR + "SubSample-InitialSegmentationResult.txt";
//	std::string GroundTruth = TESTDATADIR + "GroundTruth.txt";
//	std::string FeaturePath = TESTDATADIR + "AllFeatures.txt";
//	std::string VehicleFeaturePath = TESTDATADIR + "VehicleFeatures.txt";
//	std::string GroundFeaturePath = TESTDATADIR + "GroundFeatures.txt";
//	std::string BuildingFeaturePath = TESTDATADIR + "BuildingFeatures.txt";
//	std::string TreeFeaturePath = TESTDATADIR + "TreeFeatures.txt";
//	std::string SubsequentDir = "E:\\PointCloudDataSet\\scenes\\SceneTest\\Scene5\\";
//	std::string ManualResultPath = "E:\\PointCloudDataSet\\scenes\\SceneTest\\Scene5\\";
//
//	std::string ConfFilePath = "..\\..\\OnlineLearning\\conf\\omcb.conf";
//	Hyperparameters HP(ConfFilePath);
//	CPointCloudTool PointCloudTool;
//	std::vector<vector<double>> InitialSegmentationResult;
//
//	/************* Building Correction ***************/
//	ESemanticCategory SemanticLabel = ESemanticCategory::Building;
//	PointCloudTool.loadOriginArray(BuildingFeaturePath, InitialSegmentationResult);
//	auto InitialTrainData4BuildingBinary = InitialSegmentationResult;
//	int LabelIndex = 8;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4BuildingBinary);
//	DataSet BuildingDataSet;
//	int NumFeatures = 2, FirstFeatureIndex = 6;
//	BuildingDataSet.setNumFeatures(NumFeatures);
//	BuildingDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
//	BuildingDataSet.setLaeblIndex(LabelIndex);
//	BuildingDataSet.load(InitialTrainData4BuildingBinary);
//	Classifier* Model = new OnlineRF(HP, BuildingDataSet.m_numClasses, BuildingDataSet.m_numFeatures, BuildingDataSet.m_minFeatRange, BuildingDataSet.m_maxFeatRange);
//
//	train(Model, BuildingDataSet, HP);
//	vector<Result> Results = test(Model, BuildingDataSet, HP);
//	PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4BuildingBinary, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "BuildingApproaching.txt", InitialTrainData4BuildingBinary);
//
//	//ManualResult approaching stage 1
//	vector<string> PosManualResultFilePathVector;
//	vector<string> NegManualResultFilePathVector;
//	PosManualResultFilePathVector.push_back(TESTDATADIR + "1\\Positive.txt");
//	NegManualResultFilePathVector.push_back(TESTDATADIR + "1\\Negative.txt");
//
//	std::vector<vector<double>> ManualTrainData;
//	PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	DataSet BuildingCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
//	BuildingCorrectionOne.load(ManualTrainData);
//	train(Model, BuildingCorrectionOne, HP);
//	Results.clear();
//	Results = test(Model, BuildingDataSet, HP);
//
//	int ConfidenceColumnIndex = 9;
//	PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "1\\Result.txt", InitialSegmentationResult);
//
//	InitialSegmentationResult.clear();
//	PointCloudTool.loadOriginArray(SubsequentDir + "BuildingFeatures.txt", InitialSegmentationResult);
//	auto Test = InitialSegmentationResult;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, Test);
//	DataSet SubsequentBuildingDataSet(NumFeatures, FirstFeatureIndex, LabelIndex);
//	SubsequentBuildingDataSet.load(Test);
//	Results.clear();
//	Results = test(Model, SubsequentBuildingDataSet, HP);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(ManualResultPath + "BuildingResult.txt", InitialSegmentationResult);
//
//	delete Model;
//
//
//	/************* Tree Correction ***************/
//	////Convergence to InitialSegmentation Result
//	SemanticLabel = ESemanticCategory::Tree;
//	InitialSegmentationResult.clear();
//	PointCloudTool.loadOriginArray(TreeFeaturePath, InitialSegmentationResult);
//
//	std::vector<std::vector<double>> LastCorrectionResult;
//	PointCloudTool.loadOriginArray(TESTDATADIR + "1\\Result.txt", LastCorrectionResult);
//	LabelIndex = 9;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastCorrectionResult);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "2\\ChangeResult.txt", InitialSegmentationResult);
//	auto InitialTrainData4TreeBinary = InitialSegmentationResult;
//
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4TreeBinary);
//	DataSet TreeDataSet;
//	NumFeatures = 3, FirstFeatureIndex = 6;
//	TreeDataSet.setNumFeatures(NumFeatures);
//	TreeDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
//	TreeDataSet.setLaeblIndex(LabelIndex);
//	TreeDataSet.load(InitialTrainData4TreeBinary);
//	Model = new OnlineRF(HP, 2, NumFeatures, TreeDataSet.m_minFeatRange, TreeDataSet.m_maxFeatRange);
//	train(Model, TreeDataSet, HP);
//	Results.clear();
//	Results = test(Model, TreeDataSet, HP);
//	PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4TreeBinary, Results);
//
//	//ManualResult approaching stage 1
//	PosManualResultFilePathVector.clear();
//	NegManualResultFilePathVector.clear();
//	PosManualResultFilePathVector.push_back(TESTDATADIR + "2\\Positive.txt");
//	NegManualResultFilePathVector.push_back(TESTDATADIR + "2\\Negative.txt");
//
//	ManualTrainData.clear();
//	PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	DataSet TreeCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
//	TreeCorrectionOne.load(ManualTrainData);
//	train(Model, TreeCorrectionOne, HP);
//	Results.clear();
//	Results = test(Model, TreeDataSet, HP);
//
//	ConfidenceColumnIndex = 10;
//	PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "2\\Result.txt", InitialSegmentationResult);
//
//	InitialSegmentationResult.clear();
//	PointCloudTool.loadOriginArray(SubsequentDir + "TreeFeatures.txt", InitialSegmentationResult);
//	LastCorrectionResult.clear();
//	PointCloudTool.loadOriginArray(ManualResultPath + "BuildingResult.txt", LastCorrectionResult);
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastCorrectionResult);
//	InitialTrainData4TreeBinary.clear();
//	InitialTrainData4TreeBinary = InitialSegmentationResult;
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4TreeBinary);
//	DataSet TreeScene2DataSet(NumFeatures, FirstFeatureIndex, LabelIndex);
//	TreeScene2DataSet.load(InitialTrainData4TreeBinary);
//	Results.clear();
//	Results = test(Model, TreeScene2DataSet, HP);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(ManualResultPath + "TreeResult.txt", InitialSegmentationResult);
//
//	delete Model;
//
//	/************* Ground Correction ***************/
//	////Convergence to InitialSegmentation Result
//	SemanticLabel = ESemanticCategory::Ground;
//	InitialSegmentationResult.clear();
//	PointCloudTool.loadOriginArray(GroundFeaturePath, InitialSegmentationResult);
//
//	LastCorrectionResult.clear();
//	PointCloudTool.loadOriginArray(TESTDATADIR + "2\\Result.txt", LastCorrectionResult);
//	LabelIndex = 9;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastCorrectionResult);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "3\\ChangeResult.txt", InitialSegmentationResult);
//	auto InitialTrainData4GroundBinary = InitialSegmentationResult;
//
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4GroundBinary);
//	DataSet GroundDataSet;
//	NumFeatures = 3, FirstFeatureIndex = 6;
//	GroundDataSet.setNumFeatures(NumFeatures);
//	GroundDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
//	GroundDataSet.setLaeblIndex(LabelIndex);
//	GroundDataSet.load(InitialTrainData4GroundBinary);
//	Model = new OnlineRF(HP, 2, NumFeatures, GroundDataSet.m_minFeatRange, GroundDataSet.m_maxFeatRange);
//	train(Model, GroundDataSet, HP);
//	Results.clear();
//	Results = test(Model, GroundDataSet, HP);
//	PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4GroundBinary, Results);
//	//PointCloudTool.saveArrayFile(GroundConvergencePath, InitialTrainData4GroundBinary);
//
//	//ManualResult approaching stage 1
//	PosManualResultFilePathVector.clear();
//	NegManualResultFilePathVector.clear();
//	PosManualResultFilePathVector.push_back(TESTDATADIR + "3\\Positive.txt");
//	NegManualResultFilePathVector.push_back(TESTDATADIR + "3\\Negative.txt");
//
//	ManualTrainData.clear();
//	PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	DataSet GroundCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
//	GroundCorrectionOne.load(ManualTrainData);
//	train(Model, GroundCorrectionOne, HP);
//	Results.clear();
//	Results = test(Model, GroundDataSet, HP);
//
//	ConfidenceColumnIndex = 10;
//	PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "3\\Result.txt", InitialSegmentationResult);
//
//
//	InitialSegmentationResult.clear();
//	PointCloudTool.loadOriginArray(SubsequentDir + "GroundFeatures.txt", InitialSegmentationResult);
//	LastCorrectionResult.clear();
//	PointCloudTool.loadOriginArray(ManualResultPath + "TreeResult.txt", LastCorrectionResult);
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastCorrectionResult);
//	InitialTrainData4GroundBinary.clear();
//	InitialTrainData4GroundBinary = InitialSegmentationResult;
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4GroundBinary);
//	DataSet GroundScene2DataSet(NumFeatures, FirstFeatureIndex, LabelIndex);
//	GroundScene2DataSet.load(InitialTrainData4GroundBinary);
//	Results.clear();
//	Results = test(Model, GroundScene2DataSet, HP);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(ManualResultPath + "GroundResult.txt", InitialSegmentationResult);
//
//	delete Model;
//
//}
//
//TEST_F(CTestIndependentFeature4EachSemantic, NT_Scene3)
//{
//	std::string TESTDATADIR = "E:\\PointCloudDataSet\\scenes\\Scene2\\";
//	std::string InitialSegmentationResultPath = TESTDATADIR + "SubSample-InitialSegmentationResult.txt";
//	std::string GroundTruth = TESTDATADIR + "GroundTruth.txt";
//	std::string FeaturePath = TESTDATADIR + "AllFeatures.txt";
//	std::string VehicleFeaturePath = TESTDATADIR + "VehicleFeatures.txt";
//	std::string GroundFeaturePath = TESTDATADIR + "GroundFeatures.txt";
//	std::string BuildingFeaturePath = TESTDATADIR + "BuildingFeatures.txt";
//	std::string TreeFeaturePath = TESTDATADIR + "TreeFeatures.txt";
//	std::string SubsequentDir = "E:\\PointCloudDataSet\\scenes\\SceneTest\\Scene3\\";
//	std::string ManualResultPath = "E:\\PointCloudDataSet\\scenes\\SceneTest\\Scene3\\";
//
//	std::string ConfFilePath = "..\\..\\OnlineLearning\\conf\\omcb.conf";
//	Hyperparameters HP(ConfFilePath);
//	CPointCloudTool PointCloudTool;
//	std::vector<vector<double>> InitialSegmentationResult;
//
//	/************* Vehicle Correction ***************/
//	////Convergence to InitialSegmentation Result
//	ESemanticCategory SemanticLabel = ESemanticCategory::Vehicle;
//	PointCloudTool.loadOriginArray(VehicleFeaturePath, InitialSegmentationResult);
//	auto InitialTrainData4VehicleBinary = InitialSegmentationResult;
//	int LabelIndex = 10;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4VehicleBinary);
//	DataSet VehicleDataSet;
//	int NumFeatures = 4, FirstFeatureIndex = 6;
//	VehicleDataSet.setNumFeatures(NumFeatures);
//	VehicleDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
//	VehicleDataSet.setLaeblIndex(LabelIndex);
//	VehicleDataSet.load(InitialTrainData4VehicleBinary);
//	Classifier* Model = new OnlineRF(HP, VehicleDataSet.m_numClasses, VehicleDataSet.m_numFeatures, VehicleDataSet.m_minFeatRange, VehicleDataSet.m_maxFeatRange);
//	train(Model, VehicleDataSet, HP);
//	vector<Result> Results = test(Model, VehicleDataSet, HP);
//	PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4VehicleBinary, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "VehicileApproaching.txt", InitialTrainData4VehicleBinary);
//
//	//ManualResult approaching stage 1
//	vector<string> PosManualResultFilePathVector;
//	vector<string> NegManualResultFilePathVector;
//	PosManualResultFilePathVector.push_back(TESTDATADIR + "1\\Positive.txt");
//	NegManualResultFilePathVector.push_back(TESTDATADIR + "1\\Negative.txt");
//
//	std::vector<vector<double>> ManualTrainData;
//	PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	DataSet VehicleCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
//	VehicleCorrectionOne.load(ManualTrainData);
//	train(Model, VehicleCorrectionOne, HP);
//	Results.clear();
//	Results = test(Model, VehicleDataSet, HP);
//
//	int ConfidenceColumnIndex = 11;
//	PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "1\\Result.txt", InitialSegmentationResult);
//
//	InitialSegmentationResult.clear();
//	PointCloudTool.loadOriginArray(SubsequentDir + "VehicleFeatures.txt", InitialSegmentationResult);
//	auto Test = InitialSegmentationResult;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, Test);
//	DataSet SubsequentVehicleDataSet(NumFeatures, FirstFeatureIndex, LabelIndex);
//	SubsequentVehicleDataSet.load(Test);
//	Results.clear();
//	Results = test(Model, SubsequentVehicleDataSet, HP);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(ManualResultPath + "VehicleResult.txt", InitialSegmentationResult);
//
//	delete Model;
//
//	/************* Ground Correction ***************/
//	////Convergence to InitialSegmentation Result
//	SemanticLabel = ESemanticCategory::Ground;
//	InitialSegmentationResult.clear();
//	PointCloudTool.loadOriginArray(GroundFeaturePath, InitialSegmentationResult);
//
//	std::vector<std::vector<double>> LastCorrectionResult;
//	PointCloudTool.loadOriginArray(TESTDATADIR + "1\\Result.txt", LastCorrectionResult);
//	LabelIndex = 9;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastCorrectionResult);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "2\\ChangeResult.txt", InitialSegmentationResult);
//	auto InitialTrainData4GroundBinary = InitialSegmentationResult;
//
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4GroundBinary);
//	DataSet GroundDataSet;
//	NumFeatures = 3, FirstFeatureIndex = 6;
//	GroundDataSet.setNumFeatures(NumFeatures);
//	GroundDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
//	GroundDataSet.setLaeblIndex(LabelIndex);
//	GroundDataSet.load(InitialTrainData4GroundBinary);
//	Model = new OnlineRF(HP, GroundDataSet.m_numClasses, GroundDataSet.m_numFeatures, GroundDataSet.m_minFeatRange, GroundDataSet.m_maxFeatRange);
//	train(Model, GroundDataSet, HP);
//	Results.clear();
//	Results = test(Model, GroundDataSet, HP);
//	PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4GroundBinary, Results);
//	//PointCloudTool.saveArrayFile(GroundConvergencePath, InitialTrainData4GroundBinary);
//
//	//ManualResult approaching stage 1
//	PosManualResultFilePathVector.clear();
//	NegManualResultFilePathVector.clear();
//	PosManualResultFilePathVector.push_back(TESTDATADIR + "2\\Positive.txt");
//	NegManualResultFilePathVector.push_back(TESTDATADIR + "2\\Negative.txt");
//
//	ManualTrainData.clear();
//	PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	DataSet GroundCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
//	GroundCorrectionOne.load(ManualTrainData);
//	train(Model, GroundCorrectionOne, HP);
//	Results.clear();
//	Results = test(Model, GroundDataSet, HP);
//
//	ConfidenceColumnIndex = 10;
//	PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "2\\Result.txt", InitialSegmentationResult);
//
//
//	InitialSegmentationResult.clear();
//	PointCloudTool.loadOriginArray(SubsequentDir + "GroundFeatures.txt", InitialSegmentationResult);
//	LastCorrectionResult.clear();
//	PointCloudTool.loadOriginArray(ManualResultPath + "VehicleResult.txt", LastCorrectionResult);
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastCorrectionResult);
//	InitialTrainData4GroundBinary.clear();
//	InitialTrainData4GroundBinary = InitialSegmentationResult;
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4GroundBinary);
//	DataSet GroundScene2DataSet(NumFeatures, FirstFeatureIndex, LabelIndex);
//	GroundScene2DataSet.load(InitialTrainData4GroundBinary);
//	Results.clear();
//	Results = test(Model, GroundScene2DataSet, HP);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(ManualResultPath + "GroundResult.txt", InitialSegmentationResult);
//
//	delete Model;
//
//	/************* Tree Correction ***************/
//	////Convergence to InitialSegmentation Result
//	SemanticLabel = ESemanticCategory::Tree;
//	InitialSegmentationResult.clear();
//	PointCloudTool.loadOriginArray(TreeFeaturePath, InitialSegmentationResult);
//
//	LastCorrectionResult.clear();
//	PointCloudTool.loadOriginArray(TESTDATADIR + "2\\Result.txt", LastCorrectionResult);
//	LabelIndex = 9;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastCorrectionResult);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "3\\ChangeResult.txt", InitialSegmentationResult);
//	auto InitialTrainData4TreeBinary = InitialSegmentationResult;
//
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4TreeBinary);
//	DataSet TreeDataSet;
//	NumFeatures = 3, FirstFeatureIndex = 6;
//	TreeDataSet.setNumFeatures(NumFeatures);
//	TreeDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
//	TreeDataSet.setLaeblIndex(LabelIndex);
//	TreeDataSet.load(InitialTrainData4TreeBinary);
//	Model = new OnlineRF(HP, TreeDataSet.m_numClasses, TreeDataSet.m_numFeatures, TreeDataSet.m_minFeatRange, TreeDataSet.m_maxFeatRange);
//	train(Model, TreeDataSet, HP);
//	Results.clear();
//	Results = test(Model, TreeDataSet, HP);
//	PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4TreeBinary, Results);
//
//	//ManualResult approaching stage 1
//	PosManualResultFilePathVector.clear();
//	NegManualResultFilePathVector.clear();
//	PosManualResultFilePathVector.push_back(TESTDATADIR + "3\\Positive.txt");
//	NegManualResultFilePathVector.push_back(TESTDATADIR + "3\\Negative.txt");
//
//	ManualTrainData.clear();
//	PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	DataSet TreeCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
//	TreeCorrectionOne.load(ManualTrainData);
//	train(Model, TreeCorrectionOne, HP);
//	Results.clear();
//	Results = test(Model, TreeDataSet, HP);
//
//	ConfidenceColumnIndex = 10;
//	PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "3\\Result.txt", InitialSegmentationResult);
//
//
//	InitialSegmentationResult.clear();
//	PointCloudTool.loadOriginArray(SubsequentDir + "TreeFeatures.txt", InitialSegmentationResult);
//	LastCorrectionResult.clear();
//	PointCloudTool.loadOriginArray(ManualResultPath + "GroundResult.txt", LastCorrectionResult);
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastCorrectionResult);
//	InitialTrainData4TreeBinary.clear();
//	InitialTrainData4TreeBinary = InitialSegmentationResult;
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4TreeBinary);
//	DataSet TreeScene2DataSet(NumFeatures, FirstFeatureIndex, LabelIndex);
//	TreeScene2DataSet.load(InitialTrainData4TreeBinary);
//	Results.clear();
//	Results = test(Model, TreeScene2DataSet, HP);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(ManualResultPath + "TreeResult.txt", InitialSegmentationResult);
//
//	delete Model;
//
//}
//
//TEST_F(CTestIndependentFeature4EachSemantic, NT_Scene6)
//{
//
//	std::string TESTDATADIR = "E:\\PointCloudDataSet\\scenes\\Scene5\\";
//	std::string InitialSegmentationResultPath = TESTDATADIR + "SubSample-InitialSegmentationResult.txt";
//	std::string GroundTruth = TESTDATADIR + "GroundTruth.txt";
//	std::string FeaturePath = TESTDATADIR + "AllFeatures.txt";
//	std::string VehicleFeaturePath = TESTDATADIR + "VehicleFeatures.txt";
//	std::string GroundFeaturePath = TESTDATADIR + "GroundFeatures.txt";
//	std::string BuildingFeaturePath = TESTDATADIR + "BuildingFeatures.txt";
//	std::string TreeFeaturePath = TESTDATADIR + "TreeFeatures.txt";
//	std::string SubsequentDir = "E:\\PointCloudDataSet\\scenes\\SceneTest\\Scene2\\";
//	std::string ManualResultPath = "E:\\PointCloudDataSet\\scenes\\SceneTest\\Scene2\\";
//	std::string ConfFilePath = "..\\..\\OnlineLearning\\conf\\omcb.conf";
//	Hyperparameters HP(ConfFilePath);
//	CPointCloudTool PointCloudTool;
//	std::vector<vector<double>> InitialSegmentationResult;
//
//	/************* Vehicle Correction ***************/
//	////Convergence to InitialSegmentation Result
//	ESemanticCategory SemanticLabel = ESemanticCategory::Vehicle;
//	PointCloudTool.loadOriginArray(VehicleFeaturePath, InitialSegmentationResult);
//	auto InitialTrainData4VehicleBinary = InitialSegmentationResult;
//	int LabelIndex = 10;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4VehicleBinary);
//	DataSet VehicleDataSet;
//	int NumFeatures = 4, FirstFeatureIndex = 6;
//	VehicleDataSet.setNumFeatures(NumFeatures);
//	VehicleDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
//	VehicleDataSet.setLaeblIndex(LabelIndex);
//	VehicleDataSet.load(InitialTrainData4VehicleBinary);
//	Classifier* Model = new OnlineRF(HP, VehicleDataSet.m_numClasses, VehicleDataSet.m_numFeatures, VehicleDataSet.m_minFeatRange, VehicleDataSet.m_maxFeatRange);
//	train(Model, VehicleDataSet, HP);
//	vector<Result> Results = test(Model, VehicleDataSet, HP);
//	PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4VehicleBinary, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "VehicileApproaching.txt", InitialTrainData4VehicleBinary);
//
//	//ManualResult approaching stage 1
//	vector<string> PosManualResultFilePathVector;
//	vector<string> NegManualResultFilePathVector;
//	PosManualResultFilePathVector.push_back(TESTDATADIR + "1\\Positive.txt");
//	NegManualResultFilePathVector.push_back(TESTDATADIR + "1\\Negative.txt");
//
//	std::vector<vector<double>> ManualTrainData;
//	PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	DataSet VehicleCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
//	VehicleCorrectionOne.load(ManualTrainData);
//	train(Model, VehicleCorrectionOne, HP);
//	Results.clear();
//	Results = test(Model, VehicleDataSet, HP);
//
//	int ConfidenceColumnIndex = 11;
//	PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "1\\Result.txt", InitialSegmentationResult);
//
//
//	InitialSegmentationResult.clear();
//	PointCloudTool.loadOriginArray(SubsequentDir + "VehicleFeatures.txt", InitialSegmentationResult);
//	auto Test = InitialSegmentationResult;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, Test);
//	DataSet SubsequentVehicleDataSet(NumFeatures, FirstFeatureIndex, LabelIndex);
//	SubsequentVehicleDataSet.load(Test);
//	Results.clear();
//	Results = test(Model, SubsequentVehicleDataSet, HP);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(ManualResultPath + "VehicleResult.txt", InitialSegmentationResult);
//
//	delete Model;
//
//	/************* Ground Correction ***************/
//	////Convergence to InitialSegmentation Result
//	SemanticLabel = ESemanticCategory::Ground;
//	InitialSegmentationResult.clear();
//	PointCloudTool.loadOriginArray(GroundFeaturePath, InitialSegmentationResult);
//
//	std::vector<std::vector<double>> LastCorrectionResult;
//	PointCloudTool.loadOriginArray(TESTDATADIR + "1\\Result.txt", LastCorrectionResult);
//	LabelIndex = 9;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastCorrectionResult);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "2\\ChangeResult.txt", InitialSegmentationResult);
//	auto InitialTrainData4GroundBinary = InitialSegmentationResult;
//
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4GroundBinary);
//	DataSet GroundDataSet;
//	NumFeatures = 3, FirstFeatureIndex = 6;
//	GroundDataSet.setNumFeatures(NumFeatures);
//	GroundDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
//	GroundDataSet.setLaeblIndex(LabelIndex);
//	GroundDataSet.load(InitialTrainData4GroundBinary);
//	Model = new OnlineRF(HP, GroundDataSet.m_numClasses, GroundDataSet.m_numFeatures, GroundDataSet.m_minFeatRange, GroundDataSet.m_maxFeatRange);
//	train(Model, GroundDataSet, HP);
//	Results.clear();
//	Results = test(Model, GroundDataSet, HP);
//	PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4GroundBinary, Results);
//
//	//ManualResult approaching stage 1
//	PosManualResultFilePathVector.clear();
//	NegManualResultFilePathVector.clear();
//	PosManualResultFilePathVector.push_back(TESTDATADIR + "2\\Positive.txt");
//	NegManualResultFilePathVector.push_back(TESTDATADIR + "2\\Negative.txt");
//
//	ManualTrainData.clear();
//	PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	DataSet GroundCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
//	GroundCorrectionOne.load(ManualTrainData);
//	train(Model, GroundCorrectionOne, HP);
//	Results.clear();
//	Results = test(Model, GroundDataSet, HP);
//
//	ConfidenceColumnIndex = 10;
//	PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "2\\Result.txt", InitialSegmentationResult);
//
//
//	InitialSegmentationResult.clear();
//	PointCloudTool.loadOriginArray(SubsequentDir + "GroundFeatures.txt", InitialSegmentationResult);
//	LastCorrectionResult.clear();
//	PointCloudTool.loadOriginArray(ManualResultPath + "VehicleResult.txt", LastCorrectionResult);
//	LabelIndex = 9;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastCorrectionResult);
//	InitialTrainData4GroundBinary.clear();
//	InitialTrainData4GroundBinary = InitialSegmentationResult;
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4GroundBinary);
//	DataSet GroundScene2DataSet(NumFeatures, FirstFeatureIndex, LabelIndex);
//	GroundScene2DataSet.load(InitialTrainData4GroundBinary);
//	Results.clear();
//	Results = test(Model, GroundScene2DataSet, HP);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(ManualResultPath + "GroundResult.txt", InitialSegmentationResult);
//
//	delete Model;
//
//}
//
//TEST_F(CTestIndependentFeature4EachSemantic, NT_Scene2)
//{
//
//	std::string TESTDATADIR = "E:\\PointCloudDataSet\\scenes\\Scene1\\";
//	std::string InitialSegmentationResultPath = TESTDATADIR + "SubSample-InitialSegmentationResult.txt";
//	std::string GroundTruth = TESTDATADIR + "GroundTruth.txt";
//	std::string FeaturePath = TESTDATADIR + "AllFeatures.txt";
//	std::string VehicleFeaturePath = TESTDATADIR + "VehicleFeatures.txt";
//	std::string GroundFeaturePath = TESTDATADIR + "GroundFeatures.txt";
//	std::string BuildingFeaturePath = TESTDATADIR + "BuildingFeatures.txt";
//	std::string TreeFeaturePath = TESTDATADIR + "TreeFeatures.txt";
//	std::string SubsequentDir = "E:\\PointCloudDataSet\\scenes\\SceneTest\\Scene6\\";
//	std::string ManualResultPath = "E:\\PointCloudDataSet\\scenes\\SceneTest\\Scene6\\";
//
//	std::string ConfFilePath = "..\\..\\OnlineLearning\\conf\\omcb.conf";
//	Hyperparameters HP(ConfFilePath);
//	CPointCloudTool PointCloudTool;
//	std::vector<vector<double>> InitialSegmentationResult;
//
//	/************* Vehicle Correction ***************/
//	////Convergence to InitialSegmentation Result
//	ESemanticCategory SemanticLabel = ESemanticCategory::Vehicle;
//	PointCloudTool.loadOriginArray(VehicleFeaturePath, InitialSegmentationResult);
//	auto InitialTrainData4VehicleBinary = InitialSegmentationResult;
//	int LabelIndex = 10;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4VehicleBinary);
//	DataSet VehicleDataSet;
//	int NumFeatures = 4, FirstFeatureIndex = 6;
//	VehicleDataSet.setNumFeatures(NumFeatures);
//	VehicleDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
//	VehicleDataSet.setLaeblIndex(LabelIndex);
//	VehicleDataSet.load(InitialTrainData4VehicleBinary);
//	Classifier* Model = new OnlineRF(HP, VehicleDataSet.m_numClasses, VehicleDataSet.m_numFeatures, VehicleDataSet.m_minFeatRange, VehicleDataSet.m_maxFeatRange);
//	train(Model, VehicleDataSet, HP);
//	vector<Result> Results = test(Model, VehicleDataSet, HP);
//	PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4VehicleBinary, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "VehicileApproaching.txt", InitialTrainData4VehicleBinary);
//
//	//ManualResult approaching stage 1
//	vector<string> PosManualResultFilePathVector;
//	vector<string> NegManualResultFilePathVector;
//	PosManualResultFilePathVector.push_back(TESTDATADIR + "1\\Positive.txt");
//	NegManualResultFilePathVector.push_back(TESTDATADIR + "1\\Negative.txt");
//
//	std::vector<vector<double>> ManualTrainData;
//	PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	DataSet VehicleCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
//	VehicleCorrectionOne.load(ManualTrainData);
//	train(Model, VehicleCorrectionOne, HP);
//	Results.clear();
//	Results = test(Model, VehicleDataSet, HP);
//
//	int ConfidenceColumnIndex = 11;
//	PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "1\\Result.txt", InitialSegmentationResult);
//
//	InitialSegmentationResult.clear();
//	PointCloudTool.loadOriginArray(SubsequentDir + "VehicleFeatures.txt", InitialSegmentationResult);
//	auto Test = InitialSegmentationResult;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, Test);
//	DataSet SubsequentVehicleDataSet(NumFeatures, FirstFeatureIndex, LabelIndex);
//	SubsequentVehicleDataSet.load(Test);
//	Results.clear();
//	Results = test(Model, SubsequentVehicleDataSet, HP);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(ManualResultPath + "VehicleResult.txt", InitialSegmentationResult);
//
//	delete Model;
//
//	/************* Ground Correction ***************/
//	////Convergence to InitialSegmentation Result
//	SemanticLabel = ESemanticCategory::Ground;
//	InitialSegmentationResult.clear();
//	PointCloudTool.loadOriginArray(GroundFeaturePath, InitialSegmentationResult);
//
//	std::vector<std::vector<double>> LastCorrectionResult;
//	PointCloudTool.loadOriginArray(TESTDATADIR + "1\\Result.txt", LastCorrectionResult);
//	LabelIndex = 9;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastCorrectionResult);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "2\\ChangeResult.txt", InitialSegmentationResult);
//	auto InitialTrainData4GroundBinary = InitialSegmentationResult;
//
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4GroundBinary);
//	DataSet GroundDataSet;
//	NumFeatures = 3, FirstFeatureIndex = 6;
//	GroundDataSet.setNumFeatures(NumFeatures);
//	GroundDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
//	GroundDataSet.setLaeblIndex(LabelIndex);
//	GroundDataSet.load(InitialTrainData4GroundBinary);
//	Model = new OnlineRF(HP, GroundDataSet.m_numClasses, GroundDataSet.m_numFeatures, GroundDataSet.m_minFeatRange, GroundDataSet.m_maxFeatRange);
//	train(Model, GroundDataSet, HP);
//	Results.clear();
//	Results = test(Model, GroundDataSet, HP);
//	PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4GroundBinary, Results);
//	//PointCloudTool.saveArrayFile(GroundConvergencePath, InitialTrainData4GroundBinary);
//
//	//ManualResult approaching stage 1
//	PosManualResultFilePathVector.clear();
//	NegManualResultFilePathVector.clear();
//	PosManualResultFilePathVector.push_back(TESTDATADIR + "2\\Positive.txt");
//	NegManualResultFilePathVector.push_back(TESTDATADIR + "2\\Negative.txt");
//
//	ManualTrainData.clear();
//	PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	DataSet GroundCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
//	GroundCorrectionOne.load(ManualTrainData);
//	train(Model, GroundCorrectionOne, HP);
//	Results.clear();
//	Results = test(Model, GroundDataSet, HP);
//
//	ConfidenceColumnIndex = 10;
//	PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "2\\Result.txt", InitialSegmentationResult);
//
//
//	InitialSegmentationResult.clear();
//	PointCloudTool.loadOriginArray(SubsequentDir + "GroundFeatures.txt", InitialSegmentationResult);
//	LastCorrectionResult.clear();
//	PointCloudTool.loadOriginArray(ManualResultPath + "VehicleResult.txt", LastCorrectionResult);
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastCorrectionResult);
//	InitialTrainData4GroundBinary.clear();
//	InitialTrainData4GroundBinary = InitialSegmentationResult;
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4GroundBinary);
//	DataSet GroundScene2DataSet(NumFeatures, FirstFeatureIndex, LabelIndex);
//	GroundScene2DataSet.load(InitialTrainData4GroundBinary);
//	Results.clear();
//	Results = test(Model, GroundScene2DataSet, HP);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(ManualResultPath + "GroundResult.txt", InitialSegmentationResult);
//
//	delete Model;
//
//	/************* Tree Correction ***************/
//	////Convergence to InitialSegmentation Result
//	SemanticLabel = ESemanticCategory::Tree;
//	InitialSegmentationResult.clear();
//	PointCloudTool.loadOriginArray(TreeFeaturePath, InitialSegmentationResult);
//
//	LastCorrectionResult.clear();
//	PointCloudTool.loadOriginArray(TESTDATADIR + "2\\Result.txt", LastCorrectionResult);
//	LabelIndex = 9;
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastCorrectionResult);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "3\\ChangeResult.txt", InitialSegmentationResult);
//	auto InitialTrainData4TreeBinary = InitialSegmentationResult;
//
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4TreeBinary);
//	DataSet TreeDataSet;
//	NumFeatures = 3, FirstFeatureIndex = 6;
//	TreeDataSet.setNumFeatures(NumFeatures);
//	TreeDataSet.setFirstFeatureColumnIndex(FirstFeatureIndex);
//	TreeDataSet.setLaeblIndex(LabelIndex);
//	TreeDataSet.load(InitialTrainData4TreeBinary);
//	Model = new OnlineRF(HP, TreeDataSet.m_numClasses, TreeDataSet.m_numFeatures, TreeDataSet.m_minFeatRange, TreeDataSet.m_maxFeatRange);
//	train(Model, TreeDataSet, HP);
//	Results.clear();
//	Results = test(Model, TreeDataSet, HP);
//	PointCloudTool.visualizeORFApproachDLResult(InitialTrainData4TreeBinary, Results);
//
//	//ManualResult approaching stage 1
//	PosManualResultFilePathVector.clear();
//	NegManualResultFilePathVector.clear();
//	PosManualResultFilePathVector.push_back(TESTDATADIR + "3\\Positive.txt");
//	NegManualResultFilePathVector.push_back(TESTDATADIR + "3\\Negative.txt");
//
//	ManualTrainData.clear();
//	PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
//	DataSet TreeCorrectionOne(NumFeatures, FirstFeatureIndex, LabelIndex);
//	TreeCorrectionOne.load(ManualTrainData);
//	train(Model, TreeCorrectionOne, HP);
//	Results.clear();
//	Results = test(Model, TreeDataSet, HP);
//
//	ConfidenceColumnIndex = 10;
//	PointCloudTool.setConfidenceColumnIndex(ConfidenceColumnIndex);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(TESTDATADIR + "3\\Result.txt", InitialSegmentationResult);
//
//	InitialSegmentationResult.clear();
//	PointCloudTool.loadOriginArray(SubsequentDir + "TreeFeatures.txt", InitialSegmentationResult);
//	LastCorrectionResult.clear();
//	PointCloudTool.loadOriginArray(ManualResultPath + "GroundResult.txt", LastCorrectionResult);
//	PointCloudTool.setLabelColumnIndex(LabelIndex);
//	PointCloudTool.changeLabelAndConf2AnotherDataset(InitialSegmentationResult, LastCorrectionResult);
//	InitialTrainData4TreeBinary.clear();
//	InitialTrainData4TreeBinary = InitialSegmentationResult;
//	PointCloudTool.binarizeDLTrainData(SemanticLabel, InitialTrainData4TreeBinary);
//	DataSet TreeScene2DataSet(NumFeatures, FirstFeatureIndex, LabelIndex);
//	TreeScene2DataSet.load(InitialTrainData4TreeBinary);
//	Results.clear();
//	Results = test(Model, TreeScene2DataSet, HP);
//	PointCloudTool.visualizeORFApproachManualResult(InitialSegmentationResult, SemanticLabel, Results);
//	PointCloudTool.saveArrayFile(ManualResultPath + "TreeResult.txt", InitialSegmentationResult);
//
//	delete Model;
//
//}


//TEST_F(CTestIndependentFeature4EachSemantic, NT_removePointWithSpecifiedLabel)
//{
//	std::string CloudPath = "C:\\Users\\HPJ\\Desktop\\Paper\\DataSet\\SUM Helsinki 3D\\Suitable Test\\SUM Helsinki 3D-afterLabelCorrect\\Tile_+1986_+2691.txt";
//	auto Cloud = std::make_shared<PointCloudT>();
//	loadCloud(CloudPath, Cloud);
//
//	auto isLabelEqual255 = [](const int vLhs, const int vRhs)
//	{
//		return vLhs == vRhs;
//	};
//
//	auto isSpecifiedLabel = [&](const PointT& vPoint,std::function<bool(const int vLhs,const int vRhs)> compareFunc) {
//		return compareFunc(static_cast<std::uint32_t>(vPoint.a),255);
//	};
//
//	auto swapElement = [](PointT& vLhs, PointT& vRhs)
//	{
//		auto Temp = vLhs;
//		vLhs = vRhs;
//		vRhs = Temp;
//	};
//
//	auto CloudWithoutNegativeLabel = std::make_shared<PointCloudT>();
//	for (auto i = 0; i < Cloud->points.size();i++)
//	{
//		auto Point = Cloud->points.at(i);
//		if (!isSpecifiedLabel(Point, isLabelEqual255))
//		{
//			CloudWithoutNegativeLabel->emplace_back(Point);
//		}
//	}
//
//	auto writeCloud = [](const std::shared_ptr<PointCloudT>& vCloud, std::string SavePath)
//	{
//		std::ofstream Writer(SavePath);
//		for (auto& Point : vCloud->points)
//		{
//			Writer << Point.x << " ";
//			Writer << Point.y << " ";
//			Writer << Point.z << " ";
//			Writer << static_cast<std::uint32_t>(Point.r) << " ";
//			Writer << static_cast<std::uint32_t>(Point.g) << " ";
//			Writer << static_cast<std::uint32_t>(Point.b) << " ";
//			Writer << static_cast<std::uint32_t>(Point.a) <<"\n";
//		}
//		Writer.close();
//	};
//
//	std::string SavePath = "C:\\Users\\HPJ\\Desktop\\Paper\\DataSet\\SUM Helsinki 3D\\Suitable Test\\SUM Helsinki 3D-afterLabelCorrect\\1986_2691-CloudWithoutNegativeLabel.txt";
//	writeCloud(CloudWithoutNegativeLabel, SavePath);
//}