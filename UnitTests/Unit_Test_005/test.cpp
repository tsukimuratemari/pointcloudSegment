#include "pch.h"
#include "ProjectMap.h"
#include "ProjectMapGenerator.h"
#include "ProjectionElevation.h"

const std::string TestDataDir = "C:\\Users\\HPJ\\Desktop\\RFile\\Data\\demo\\A7\\Point\\A7GT\\";

class Test_CProjectElevation :public testing::Test
{
protected:
	void SetUp() override
	{
		;
	}

	void TearDown() override
	{
		;
	}
};

TEST_F(Test_CProjectElevation, NT_generateMap) {

	PointCloudT::Ptr Cloud(new(PointCloudT));
	Cloud->emplace_back(0, 0, 0);
	Cloud->emplace_back(0.5, 0.5, 0.5);
	Cloud->emplace_back(2, 2, 1.5);
	Cloud->emplace_back(0.5, 0.5, 0.7);
	Cloud->emplace_back(0.5, 0.5, 0.8);
	Cloud->emplace_back(0.5, 0.5, 0.9);

	CProjectMap ProjectMap;
	CProjectMapGenerator MapGenerator(Cloud);
	int Width = 2, Height = 2;
	MapGenerator.generate(Width, Height);
	MapGenerator.dumpProjectMap(ProjectMap);

	EXPECT_EQ(ProjectMap.getValueAt(0, 0).size(), 5);
	EXPECT_EQ(ProjectMap.getValueAt(0, 1).size(), 0);
	EXPECT_EQ(ProjectMap.getValueAt(1, 0).size(), 0);
	EXPECT_EQ(ProjectMap.getValueAt(1, 1).size(), 1);
};

#ifdef _DEBUG
TEST_F(Test_CProjectElevation, DT_generateMap_DeathTest) {

	PointCloudT::Ptr Cloud(new(PointCloudT));

	CProjectMap ProjectMap;
	CProjectMapGenerator MapGenerator(Cloud);

	int Width = 2, Height = 2;
	EXPECT_DEATH(MapGenerator.generate(Width, Height), "");

	Cloud->emplace_back(0, 0, 0);
	Cloud->emplace_back(0.5, 0.5, 0.5);
	Cloud->emplace_back(2, 2, 1.5);
	Cloud->emplace_back(0.5, 0.5, 0.7);
	Cloud->emplace_back(0.5, 0.5, 0.8);
	Cloud->emplace_back(0.5, 0.5, 0.9);

	Width = -1, Height = 10;
	EXPECT_DEATH(MapGenerator.generate(Width, Height), "");

	Width = 0, Height = 0;
	EXPECT_DEATH(MapGenerator.generate(Width, Height), "");
}
#endif // _DEBUG

TEST_F(Test_CProjectElevation, NT_computeMaxEvelationOfClusters)
{
	PointCloudT::Ptr Cloud(new(PointCloudT));
	Cloud->emplace_back(0.0f, 0.0f, 0.0f);
	Cloud->emplace_back(0.5f, 0.5f, 0.0f);
	Cloud->emplace_back(0.5f, 0.5f, 0.7f);
	Cloud->emplace_back(0.5f, 0.5f, 0.8f);
	Cloud->emplace_back(0.5f, 0.5f, 0.9f);
	int Width = 2, Height = 2;
	CProjectionElevation Computor(Cloud, Width, Height);
	Computor.setMaxAllowedDistanceWithinACluster(1.0f);

	auto Results=Computor.computeMaxEvelationOfClusters(*Cloud);
	EXPECT_EQ(Results.size(), 1);
	EXPECT_EQ(Results[0], 0.9f);

	Cloud->clear();
	Cloud->emplace_back(2.0f, 2.0f, 0.5f);
	Cloud->emplace_back(1.5f, 1.5f, 0.7f);
	Cloud->emplace_back(1.6f, 1.6f, 1.8f);
	Cloud->emplace_back(1.7f, 1.7f, 1.9f);
	Results = Computor.computeMaxEvelationOfClusters(*Cloud);
	EXPECT_EQ(Results.size(), 2);
	EXPECT_EQ(Results[0], 0.7f);
	EXPECT_EQ(Results[1], 1.9f);
}

TEST_F(Test_CProjectElevation, NT_computeProjectEvelation)
{
	PointCloudT::Ptr Cloud(new(PointCloudT));
	Cloud->emplace_back(0, 0, 0);
	Cloud->emplace_back(0.5, 0.5, 0.5);
	Cloud->emplace_back(2, 2, 1.5);
	Cloud->emplace_back(0.5, 0.5, 0.7);
	Cloud->emplace_back(0.5, 0.5, 1.8);
	Cloud->emplace_back(0.5, 0.5, 1.9);
	int Width = 2, Height = 2;
	CProjectionElevation Computor(Cloud, Width, Height);
	std::unordered_map<int, float> Results;
	Computor.compute(Results);

	EXPECT_EQ(Results.at(0), 0.7f);
	EXPECT_EQ(Results.at(1), 0.7f);
	EXPECT_EQ(Results.at(2), 1.5f);
	EXPECT_EQ(Results.at(3), 0.7f);
	EXPECT_EQ(Results.at(4), 1.9f);
	EXPECT_EQ(Results.at(5), 1.9f);
}

TEST_F(Test_CProjectElevation, NT_UseRealData)
{
	PointCloudT::Ptr Cloud(new(PointCloudT));
	loadCloud(TestDataDir + "A7GT.txt", Cloud);
	
	auto Start = std::chrono::steady_clock::now();
	int Width = 128, Height = 128;
	CProjectionElevation Computor(Cloud, Width, Height);
	std::unordered_map<int, float> Results;
	Computor.compute(Results);
	auto End = std::chrono::steady_clock::now();
	auto Duration = std::chrono::duration_cast<std::chrono::milliseconds>(End - Start);
	std::cout << "cost time(ms):" << Duration.count() << std::endl;

	EXPECT_EQ(Results.size(), Cloud->size());

	std::ofstream Writer(TestDataDir + "ProjectEvelation.txt");
	int Count = 0;
	for (auto& Point : *Cloud)
	{
		Writer << Point.x<<" ";
		Writer << Point.y << " ";
		Writer << Point.z << " ";
		Writer << (int)Point.r << " ";
		Writer << (int)Point.g << " ";
		Writer << (int)Point.b << " ";
		Writer << (int)Point.a << " ";
		Writer << Results.at(Count++) << "\n";
	}
	Writer.close();
}




