#include "pch.h"
#include "Common.h"
#include "RelativeElevation.h"

const std::string DivideCloud2Cells_TestFile = "..\\UnitTestData\\Unit_Test_001\\DivideCloud2Cells_UnitTest.ply";
const std::string comAveElevationAndAbsGroundValue_TestFile = "..\\UnitTestData\\Unit_Test_001\\comAveElevationAndAbsGroundValue_UnitTest.ply";

class TEST_CRelativeElevation :public testing::Test
{
public:
	void SetUp() override
	{

	}

	void TearDown() override
	{

	}
};

TEST_F(TEST_CRelativeElevation, NT_divideCloud2Cells)
{
	PointCloudT::Ptr Cloud(new PointCloudT());
	if (pcl::io::loadPLYFile<PointT>(DivideCloud2Cells_TestFile, *Cloud))
	{
		pcl::console::print_error("Error loading cloud file!\n");
		EXPECT_TRUE(0);
	}

	std::uint32_t CellScale = 10;
	std::uint32_t KernelScale = 5;
	CRelativeElevation RelativeElevationOperator(Cloud, CellScale, KernelScale);

	std::map<std::uint32_t, SCell> Cells;
	RelativeElevationOperator.divideCloud2Cells(Cells);

	EXPECT_EQ(Cells.size(), 5);
	EXPECT_EQ(Cells.at(0)._cloud.size(), 2);
	EXPECT_EQ(Cells.at(9)._cloud.size(), 2);
	EXPECT_EQ(Cells.at(55)._cloud.size(), 4);
	EXPECT_EQ(Cells.at(90)._cloud.size(), 2);
	EXPECT_EQ(Cells.at(99)._cloud.size(), 2);
};

TEST_F(TEST_CRelativeElevation, NT_getCellNeighbourIds)
{
	//the schematic diagram of cells' distridution can be found in the path "..\Unit_Test_Data\Unit_Test_001";
	std::vector<int> CellIds = {
	0,9,
	20,22,23,24,25,26,29,
	32,33,34,35,36,
	42,43,44,45,46,
	50,52,53,54,55,56,59,
	62,63,64,65,66,
	70,79,
	90,99 };

	std::map<std::uint32_t, SCell> Cells;
	for (auto Id : CellIds)
	{
		SCell Cell;
		Cell._Id = Id;
		Cells.insert(std::make_pair(Id, Cell));
	}

	std::uint32_t CellScale = 10;
	std::uint32_t KernelScale = 5;
	CRelativeElevation RelativeElevationOperator;
	RelativeElevationOperator.setCellScale(CellScale);
	RelativeElevationOperator.setKernelScale(KernelScale);

	std::vector<std::uint32_t> NeighbourIds;
	RelativeElevationOperator.getCellNeighbourIds(Cells, Cells.find(0)->second, NeighbourIds);
	std::vector<std::uint32_t> ActualNeighbourIds4Cell0 = { 0,20,22 };
	EXPECT_EQ(NeighbourIds, ActualNeighbourIds4Cell0);

	NeighbourIds.clear();
	RelativeElevationOperator.getCellNeighbourIds(Cells, Cells.find(9)->second, NeighbourIds);
	std::vector<std::uint32_t> ActualNeighbourIds4Cell9 = { 9,29 };
	EXPECT_EQ(NeighbourIds, ActualNeighbourIds4Cell9);

	NeighbourIds.clear();
	RelativeElevationOperator.getCellNeighbourIds(Cells, Cells.find(44)->second, NeighbourIds);
	std::vector<std::uint32_t> ActualNeighbourIds4Cell44 = { 22,32,42,52,62,23,33,43,53,63,24,34,44,54,64,25,35,45,55,65,26,36,46,56,66 };
	EXPECT_EQ(NeighbourIds, ActualNeighbourIds4Cell44);

	NeighbourIds.clear();
	RelativeElevationOperator.getCellNeighbourIds(Cells, Cells.find(50)->second, NeighbourIds);
	std::vector<std::uint32_t> ActualNeighbourIds4Cell50 = { 50,70,32,42,52,62 };
	EXPECT_EQ(NeighbourIds, ActualNeighbourIds4Cell50);

	NeighbourIds.clear();
	RelativeElevationOperator.getCellNeighbourIds(Cells, Cells.find(59)->second, NeighbourIds);
	std::vector<std::uint32_t> ActualNeighbourIds4Cell59 = { 59,79 };
	EXPECT_EQ(NeighbourIds, ActualNeighbourIds4Cell59);

	NeighbourIds.clear();
	RelativeElevationOperator.getCellNeighbourIds(Cells, Cells.find(90)->second, NeighbourIds);
	std::vector<std::uint32_t> ActualNeighbourIds4Cell90 = { 70,90 };
	EXPECT_EQ(NeighbourIds, ActualNeighbourIds4Cell90);

	NeighbourIds.clear();
	RelativeElevationOperator.getCellNeighbourIds(Cells, Cells.find(99)->second, NeighbourIds);
	std::vector<std::uint32_t> ActualNeighbourIds4Cell99 = { 79,99 };
	EXPECT_EQ(NeighbourIds, ActualNeighbourIds4Cell99);
};

TEST_F(TEST_CRelativeElevation, NT_comAveElevationAndAbsGroundValue)
{
	PointCloudT::Ptr Cloud(new PointCloudT());
	if (pcl::io::loadPLYFile<PointT>(comAveElevationAndAbsGroundValue_TestFile, *Cloud))
	{
		pcl::console::print_error("Error loading cloud file!\n");
		EXPECT_TRUE(0);
	}

	std::uint32_t CellScale = 10;
	std::uint32_t KernelScale = 5;
	CRelativeElevation RelativeElevationOperator(Cloud, CellScale, KernelScale);

	std::map<std::uint32_t, SCell> Cells;
	RelativeElevationOperator.divideCloud2Cells(Cells);

	std::map<std::uint32_t, float> AveElevations;
	std::map<std::uint32_t, float> AbsGroundValues;
	RelativeElevationOperator.comAveElevationAndAbsGroundValue(Cells, AveElevations, AbsGroundValues);
	//may be has the problem about float precision
	EXPECT_EQ(AveElevations.size(), 5);
	EXPECT_EQ(AveElevations.at(0), 0.25f);
	EXPECT_EQ(AveElevations.at(9), 0.25f);
	EXPECT_EQ(AveElevations.at(55), 5.45f);
	EXPECT_EQ(AveElevations.at(90), 9.75f);
	EXPECT_EQ(AveElevations.at(99), 9.75f);

	EXPECT_EQ(AbsGroundValues.size(), 5);
	EXPECT_EQ(AbsGroundValues.at(0), 0.0f);
	EXPECT_EQ(AbsGroundValues.at(9), 0.0f);
	EXPECT_EQ(AbsGroundValues.at(55), 5.0f);
	EXPECT_EQ(AbsGroundValues.at(90), 9.5f);
	EXPECT_EQ(AbsGroundValues.at(99), 9.5f);
};

TEST_F(TEST_CRelativeElevation, NT_IsSatisfyGaussianDistribution)
{
	SCell CellThatSatisfyGaussianDistribution;
	CellThatSatisfyGaussianDistribution._Id = 1;
	std::default_random_engine Gen;
	float ZDefault = 0.1;
	float StanderdDerivation = 0.02;
	std::normal_distribution<float> Distrition(ZDefault, StanderdDerivation);
	float AvearageElevation = 0;
	for (std::uint32_t i = 0; i < 100; i++)
	{
		float RandomValue = Distrition(Gen);
		CellThatSatisfyGaussianDistribution._cloud.push_back(PointT(0, 0, RandomValue));
		AvearageElevation += RandomValue;
	}

	SCell CellThatNonSatisfyGaussianDistribution;
	CellThatNonSatisfyGaussianDistribution._Id = 2;
	CellThatNonSatisfyGaussianDistribution._cloud.push_back(PointT(0, 0, 1.0f));
	CellThatNonSatisfyGaussianDistribution._cloud.push_back(PointT(0, 0, 1.0f));
	CellThatNonSatisfyGaussianDistribution._cloud.push_back(PointT(0, 0, 1.0f));
	CellThatNonSatisfyGaussianDistribution._cloud.push_back(PointT(0, 0, 1.0f));
	CellThatNonSatisfyGaussianDistribution._cloud.push_back(PointT(0, 0, 1.0f));

	std::map<std::uint32_t, float> AveElevations;
	AveElevations.insert(std::make_pair(1, AvearageElevation/100));
	AveElevations.insert(std::make_pair(2, 1.0f));

	CRelativeElevation RelativeElevationOperator;
	EXPECT_TRUE(RelativeElevationOperator.IsSatisfyGaussianDistribution(CellThatSatisfyGaussianDistribution, AveElevations));
	EXPECT_FALSE(RelativeElevationOperator.IsSatisfyGaussianDistribution(CellThatNonSatisfyGaussianDistribution, AveElevations));
}
	