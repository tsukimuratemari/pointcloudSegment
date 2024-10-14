#include "pch.h"
#include "ComputeAcc.h"
class TEST_ComputeAcc :public testing::Test
{
protected:
    void SetUp() override
    {

    }

    void TearDown() override
    {

    }


};



TEST(TEST_ComputeAcc, ORFAcc)
{
    std::string FilePath = "E:\\PointCloudDataSet\\sum\\1\\";
    std::string PointCloudPath = FilePath + "6\\PostPro.txt";
    std::string GTPath = FilePath + "origin.txt";
    CAccComputer Computer;
    std::vector<int> DLResult;
    Computer.computeAcc(PointCloudPath, GTPath);

        //for (int k = 1; k <= 6; k++)
        //{
        //    std::string PrediectedResultPath = "E:\\PointCloudDataSet\\scenes\\Stage2_Segmentation_NewScene\\Scene" + std::to_string(k) + "\\PostPro.txt";
        //    std::string GTResultPath = "E:\\PointCloudDataSet\\scenes\\Stage1_Segmentation_NewScene\\Scene" + std::to_string(k) + "\\Origin.txt";
        //    CAccComputer Computer;
        //    Computer.computeAcc(PrediectedResultPath, GTResultPath);
        //}
}