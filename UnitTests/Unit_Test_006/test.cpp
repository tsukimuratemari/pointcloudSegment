#include "pch.h"
#include "ElevationDifference.h"

const std::string TestDataDir = "C:\\Users\\HPJ\\Desktop\\RFile\\Data\\demo\\A7\\Point\\A7GT\\";

TEST(TestCase_CElevationDifference, NT_computeElevationDifferenceWithIdealData) {
	PointCloudT::Ptr Cloud(new(PointCloudT));
	Cloud->emplace_back(0.0f, 0.0f, 0.0f);
	Cloud->emplace_back(0.5f, 0.5f, 0.5f);
	Cloud->emplace_back(2.0f, 2.0f, 1.5f);
	Cloud->emplace_back(0.5f, 0.5f, 0.7f);
	Cloud->emplace_back(0.5f, 0.5f, 0.8f);
	Cloud->emplace_back(0.5f, 0.5f, 0.9f);

	std::shared_ptr<CElevationDifference> ElevationDiffer=std::make_shared<CElevationDifference>(Cloud);
	std::unordered_map<int, float> ElevationDiffs;
	ElevationDiffer->compute(ElevationDiffs);

	EXPECT_EQ(ElevationDiffs.size(), Cloud->size());
	EXPECT_EQ(ElevationDiffs.at(0),0.0f);
	EXPECT_EQ(ElevationDiffs.at(1), 0.5f);
	EXPECT_EQ(ElevationDiffs.at(2), 0.0f);
	EXPECT_EQ(ElevationDiffs.at(3), 0.7f);
	EXPECT_EQ(ElevationDiffs.at(4), 0.8f);
	EXPECT_EQ(ElevationDiffs.at(5), 0.9f);
}

TEST(TestCase_CElevationDifference, NT_computeElevationDifferenceWithRealData) {
	PointCloudT::Ptr Cloud(new PointCloudT);
	loadCloud(TestDataDir + "A7GT.txt", Cloud);

	std::shared_ptr<CElevationDifference> ElevationDiffer = std::make_shared<CElevationDifference>(Cloud);
	std::unordered_map<int, float> ElevationDiffs;
	ElevationDiffer->setRadius(3.0f);
	ElevationDiffer->computeBasedProjectElevation(ElevationDiffs);
	EXPECT_EQ(ElevationDiffs.size(), Cloud->size());

	std::ofstream Writer(TestDataDir + "EvelationDifference.txt");
	int Count = 0;
	for (auto& Point : *Cloud)
	{
		Writer << Point.x << " ";
		Writer << Point.y << " ";
		Writer << Point.z << " ";
		Writer << (int)Point.r << " ";
		Writer << (int)Point.g << " ";
		Writer << (int)Point.b << " ";
		Writer << (int)Point.a << " ";
		Writer << ElevationDiffs.at(Count++) << "\n";
	}
	Writer.close();
}