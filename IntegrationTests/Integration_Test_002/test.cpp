#include "pch.h"
#include "PCTool.h"





class CTestIndependentFeature4EachSemantic :public testing::Test
{
public:

	CPCTool Tool;

	void SetUp() override
	{
		Tool = CPCTool();
	}

	void TearDown() override
	{

	}

};

//TEST_F(CTestIndependentFeature4EachSemantic, FeatureExtrat)
//{
//	for (int i = 0; i < 30; i++)
//	{
//		std::string IntialDir = "E:\\PointCloudDataSet\\sum\\Scene";
//		IntialDir += std::to_string(i) + "\\Scene" + std::to_string(i) + ".txt";
//		Extractor.setInitialPath(IntialDir);
//		Extractor.extractFeatures();
//	}
//}


TEST_F(CTestIndependentFeature4EachSemantic, Scene0)
{
	std::vector<std::pair<std::string, std::vector<ESemanticCategory>>> TrainDir;
	std::vector<std::vector<std::string>> TestDir;
	int UsedModelNum = 5;
	std::vector<int> ModelIndex = { 15,0,16,19,1,3,14,2,18,8,11,7 };
	//明天把顺序改成15,19,0,16,1,3,14,2,18,8,11,7 
	//std::vector<int> ModelIndex = { 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29 };
	std::vector<std::vector<int>>TestSceneIndex
	{
		{ },
		{ },
		{ },
		{ },
		{ 2,3,4,5,6,7,8,9,10,11,12,14,17,18}
	};

	for (int i = 0; i < UsedModelNum; i++)
	{
		std::string IntialDir = "E:\\PointCloudDataSet\\sum\\Scene" + std::to_string(ModelIndex[i]) + "\\Scene" + std::to_string(ModelIndex[i]) + ".txt";
		TrainDir.push_back(std::pair<std::string, std::vector<ESemanticCategory>>(IntialDir, SCENESEMINFOLIST[ModelIndex[i]]));
	}
	for (int i = 0; i < TestSceneIndex.size(); i++)
	{
		std::vector<std::string> TestScenes;
		for (int k = 0; k < TestSceneIndex[i].size(); k++)
		{
			std::string TESTDIR = "E:\\PointCloudDataSet\\sum\\Scene" + std::to_string(TestSceneIndex[i][k]) + "\\";
			TestScenes.push_back(TESTDIR);	
		}
		TestDir.push_back(TestScenes);
	}

	Tool.trainORFModel(TrainDir, TestDir);

	//Compute Acc
	for (int i = 0; i < TestDir.size(); i++)
	{
		std::cout << "Model" << std::to_string(i) << ": " << endl;
		for (int k = 0; k < TestDir[i].size(); k++)
		{
			std::vector<std::vector<double>> FinalResult;
			std::vector<std::vector<double>> GT;
			std::string FinFileName = TestDir[i][k] + Tool.returnSemanticString(TrainDir[i].second.back()) + "Result.txt";
			std::string GTFileName = TestDir[i][k].substr(0, TestDir[i][k].find_last_of('\\'));
			
			GTFileName = TestDir[i][k] + GTFileName.substr(GTFileName.find_last_of('\\') + 1) + ".txt";
			//PostProcessing
	
			std::string PostProFileName = TestDir[i][k] + "PostPro.txt";
		
			CSmallScalePostProcessing SmallScalePostProcessing(FinFileName);
			//std::cout << "PostPro前：" << std::endl;
			//SmallScalePostProcessing.computeAcc(SmallScalePostProcessing.pPointCloud, GTFileName);
			std::vector<std::vector<int>> ClusterSet;
			SmallScalePostProcessing.clusteringPointCloud(ClusterSet);
			SmallScalePostProcessing.savePointCloud(PostProFileName);
			//std::cout << "PostPro后：" << std::endl;
			SmallScalePostProcessing.computeAcc(SmallScalePostProcessing.pPointCloud, GTFileName);
			

		}
		std::cout << endl;
		
	}
		
	




}
