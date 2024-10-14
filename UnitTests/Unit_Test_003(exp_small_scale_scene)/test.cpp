#include "pch.h"
#include "SmallScalePostProcessing.h"
class TEST_PostProcessing :public testing::Test
{
protected:
    void SetUp() override
    {

    }

    void TearDown() override
    {

    }
};



//TEST(NT_SmallScalePostProcessing, FirstSem)
//{
//    std::string FilePath = "E:\\PointCloudDataSet\\sum\\Scene0\\";
//    std::string OriginFilePath = FilePath + "BuildingResult.txt";
//    std::string OutputFilePath = FilePath + "FirstStage.txt";
//
//    CSmallScalePostProcessing SmallScalePostProcessing(OriginFilePath, 8);
//    std::vector<std::vector<int>> ClusterSet;
//    SmallScalePostProcessing.clusteringPointCloud(ClusterSet);
//    SmallScalePostProcessing.savePointCloud(OutputFilePath);
//}

TEST(NT_SmallScalePostProcessing, SecondSem)
{
    std::string FilePath = "E:\\desktop\\Paper\\Exp\\Scene1\\subsample_cloud\\Randlanet\\Interactions\\1\\";
    std::string OriginFilePath = FilePath + "Result.txt";
    std::string OutputFilePath = FilePath + "PostPro.txt";

    CSmallScalePostProcessing SmallScalePostProcessing(OriginFilePath, 10);
    std::vector<std::vector<int>> ClusterSet;
    SmallScalePostProcessing.clusteringPointCloud(ClusterSet);
    SmallScalePostProcessing.savePointCloud(OutputFilePath);
}