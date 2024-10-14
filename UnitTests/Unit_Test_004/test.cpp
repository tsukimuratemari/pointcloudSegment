#include "pch.h"
#include "data.h"
#include "Utils.h"
#include "Common.h"
#include "experimenter.h"
#include "online_rf.h"

////the dir of super voxel parameters "003-009"
//const std::string TESTDATA_DIR = "C:\\Users\\HPJ\\Desktop\\RFile\\Data\\demo\\A7\\003-009\\A7GT\\";

////the dir of super voxel parameters "001-002"
//const std::string TESTDATA_DIR = "C:\\Users\\HPJ\\Desktop\\RFile\\Data\\demo\\A7\\001-002\\A7GT\\";

////the dir of point based FeatureSelection
const std::string TESTDATA_DIR = "C:\\Users\\HPJ\\Desktop\\RFile\\Data\\demo\\A7\\Point\\A7GT\\";

//TEST(Exp_confirmUpperBound, getBinaryClassificationResults) {
//    CPointCloudTool PointCloudTool;
//    vector<vector<double>> InitialSegmentationResult;
//    string InitialSegmentationResultFilePath = TESTDATA_DIR+"SVofRegion.txt";
//    PointCloudTool.loadOriginArray(InitialSegmentationResultFilePath, InitialSegmentationResult);
//    std::string ConfFilePath = "..\\..\\OnlineLearning\\conf\\omcb.conf";
//    Hyperparameters HP(ConfFilePath);
//    DataSet TestData(InitialSegmentationResult);
//
//    auto getBinaryClassificationConvergenceResult = [&](vector<vector<double>> vInitalResult, ESemanticCategory vSemanticLabel,std::string vSaveFileName)
//    {
//        PointCloudTool.binarizeDLTrainData(vSemanticLabel, vInitalResult);
//        DataSet TrainData(vInitalResult);
//
//        Classifier* Model = new OnlineRF(HP, TrainData.m_numClasses, TrainData.m_numFeatures, TrainData.m_minFeatRange, TrainData.m_maxFeatRange);
//        train(Model, TrainData, HP);
//        vector<Result> Results = test(Model, TestData, HP);
//        PointCloudTool.visualizeORFApproachDLResult(vInitalResult, Results);
//        PointCloudTool.saveArrayFile(TESTDATA_DIR +vSaveFileName, vInitalResult);
//        delete Model;
//    };
//
//    getBinaryClassificationConvergenceResult(InitialSegmentationResult, ESemanticCategory::Building, "BuildingConvergenceResult.txt");
//    getBinaryClassificationConvergenceResult(InitialSegmentationResult, ESemanticCategory::Vehicle, "VehicleConvergenceResult.txt");
//    getBinaryClassificationConvergenceResult(InitialSegmentationResult, ESemanticCategory::Ground, "GroundConvergenceResult.txt");
//    getBinaryClassificationConvergenceResult(InitialSegmentationResult, ESemanticCategory::Tree, "TreeConvergenceResult.txt");
//}
//
//TEST(Exp_confirmUpperBound, combine2WholeSceneInheritanceResult)
//{
//    auto __loadBCResult = [](const std::string& vSVCloudFilePath, ESemanticCategory& vSemanticLabel, std::unordered_multimap<std::uint32_t, ESemanticCategory>& voBCResults)
//    {
//        std::ifstream Reader(vSVCloudFilePath);
//        std::string Line;
//        while (std::getline(Reader, Line))
//        {
//            std::stringstream SS(Line);
//            std::string TMP;
//            PointT Point;
//            std::vector<double> SingleRowData;
//            while (std::getline(SS, TMP, ' '))
//            {
//                SingleRowData.push_back(std::stod(TMP));
//            }
//            ESemanticCategory Label = static_cast<EBiClassificationSemanticCategory>(SingleRowData[12]) == EBiClassificationSemanticCategory::PositiveCategory ? vSemanticLabel : ESemanticCategory::Undefined;
//            voBCResults.emplace(static_cast<std::uint32_t>(SingleRowData[11]), Label);
//        }
//        Reader.close();
//    };
//    auto __combineBCResults = [](std::unordered_multimap<std::uint32_t, ESemanticCategory>& vBCResults, std::unordered_map<std::uint32_t, ESemanticCategory>& voMCResult)
//    {
//        for (auto Itr = vBCResults.cbegin(); Itr != vBCResults.cend(); ++Itr)
//        {
//            std::pair<std::uint32_t, ESemanticCategory> MCResult(*Itr);
//            auto BCResults = vBCResults.equal_range(Itr->first);
//            for (auto ResultItr = BCResults.first; ResultItr != BCResults.second; ++ResultItr)
//            {
//                //遭遇多结果冲突时，随机选取一个结果（实际上是取数据结构中最后一个结果，但因为unordered_map无法控制存储顺序，故相当于随机）;
//                if (ResultItr->second != ESemanticCategory::Undefined)
//                    MCResult = *ResultItr;
//                ++Itr;
//            }
//            voMCResult.emplace(MCResult);
//        }
//    };
//    auto __loadCloudAndStoreMCResult = [&](const std::string& vCloudFilePath, const std::string& vSaveFilePath, std::unordered_map<std::uint32_t, ESemanticCategory>& vMCResult)
//    {
//        std::ifstream Reader(vCloudFilePath);
//        std::ofstream Writer(vSaveFilePath);
//        std::string Line;
//        while (std::getline(Reader, Line))
//        {
//            std::stringstream SS(Line);
//            std::string TMP;
//            std::vector<double> SingleRowData;
//            while (std::getline(SS, TMP, ' '))
//            {
//                SingleRowData.push_back(std::stod(TMP));
//            }
//            Writer << SingleRowData[0] << " ";
//            Writer << SingleRowData[1] << " ";
//            Writer << SingleRowData[2] << " ";
//            Writer << static_cast<std::uint32_t>(SingleRowData[3]) << " ";
//            Writer << static_cast<std::uint32_t>(SingleRowData[4]) << " ";
//            Writer << static_cast<std::uint32_t>(SingleRowData[5]) << " ";
//            Writer << static_cast<double>(vMCResult.at(SingleRowData[11])) << "\n";
//        }
//        Reader.close();
//        Writer.close();
//    };
//
//    std::string BCResultFilePath = TESTDATA_DIR+"BuildingConvergenceResult.txt";
//    ESemanticCategory BCPositiveLabel = ESemanticCategory::Building;
//    std::unordered_multimap<std::uint32_t, ESemanticCategory> BCResults;
//    __loadBCResult(BCResultFilePath, BCPositiveLabel, BCResults);
//
//    BCResultFilePath = TESTDATA_DIR+"TreeConvergenceResult.txt";
//    BCPositiveLabel = ESemanticCategory::Tree;
//    __loadBCResult(BCResultFilePath, BCPositiveLabel, BCResults);
//
//    BCResultFilePath = TESTDATA_DIR+"GroundConvergenceResult.txt";
//    BCPositiveLabel = ESemanticCategory::Ground;
//    __loadBCResult(BCResultFilePath, BCPositiveLabel, BCResults);
//
//    BCResultFilePath = TESTDATA_DIR+"VehicleConvergenceResult.txt";
//    BCPositiveLabel = ESemanticCategory::Vehicle;
//    __loadBCResult(BCResultFilePath, BCPositiveLabel, BCResults);
//
//    std::unordered_map<std::uint32_t, ESemanticCategory> MCResult;
//    //__combineBCResults(BCResults, MCResult);
//
//    for (auto& Element : BCResults)
//    {
//        if (!MCResult.contains(Element.first))
//        {
//            MCResult.insert(Element);
//        }
//        else if (MCResult.at(Element.first) == ESemanticCategory::Undefined && Element.second != ESemanticCategory::Undefined)
//        {
//            MCResult.at(Element.first) = Element.second;
//        }
//    }
//
//    const std::string SVCloudFilePath = BCResultFilePath;
//    const std::string SaveFilePath = TESTDATA_DIR+"MConvergenceResult.txt";
//    __loadCloudAndStoreMCResult(SVCloudFilePath, SaveFilePath, MCResult);
//}

//TEST(Exp_confirmUpperBound, mapSegmentation2OriginPointcloud)
//{
//    auto loadSVCloud = [&](const std::string& vSVCloudFilePath, PointCloudT::Ptr& voSVCloud,std::uint32_t vLabelColumnIndex)
//    {
//        std::ifstream Reader(vSVCloudFilePath);
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
//            Point.r = static_cast<std::uint8_t>(String2Double[3]);
//            Point.g = static_cast<std::uint8_t>(String2Double[4]);
//            Point.b = static_cast<std::uint8_t>(String2Double[5]);
//            Point.a = static_cast<std::uint8_t>(String2Double[vLabelColumnIndex]);
//            voSVCloud->emplace_back(Point);
//        }
//        Reader.close();
//    };
//
//    auto normalizePointCloud = [&](PointCloudT::Ptr& vioOriginCloud)
//    {
//        std::pair<pcl::PointXYZ, pcl::PointXYZ> MinMaxCoordinate;
//        comMinMax4XYZ(vioOriginCloud, MinMaxCoordinate);
//
//        float XLen = MinMaxCoordinate.second.x - MinMaxCoordinate.first.x;
//        float YLen = MinMaxCoordinate.second.y - MinMaxCoordinate.first.y;
//        float ZLen = MinMaxCoordinate.second.z - MinMaxCoordinate.first.z;
//        float XYScale = XLen / YLen;
//        float ZYScale = ZLen / YLen;
//        for (std::uint32_t i = 0; i < vioOriginCloud->points.size(); i++)
//        {
//            vioOriginCloud->points[i].x = (vioOriginCloud->points[i].x - MinMaxCoordinate.first.x) / XLen * XYScale;
//            vioOriginCloud->points[i].y = (vioOriginCloud->points[i].y - MinMaxCoordinate.first.y) / YLen;
//            vioOriginCloud->points[i].z = (vioOriginCloud->points[i].z - MinMaxCoordinate.first.z) / ZLen * ZYScale;
//        }
//    };
//
//    auto __checkLabel = [](std::map<std::uint32_t, std::uint32_t>& vLabelCount, const std::uint32_t vLabel)
//    {
//        if (vLabelCount.contains(vLabel))
//        {
//            vLabelCount.at(vLabel)++;
//        }
//        else
//        {
//            vLabelCount.insert(std::make_pair(vLabel, 1));
//        }
//    };
//
//    auto __findMaxCountLabel = [](const std::map<std::uint32_t, std::uint32_t>& vLabelCount)
//    {
//        std::pair<std::uint32_t, std::uint32_t> MaxCountLabel(0, 0);
//        for (auto& Label : vLabelCount)
//        {
//            if (Label.second >= MaxCountLabel.second)
//                MaxCountLabel = Label;
//        }
//        return MaxCountLabel.first;
//    };
//
//    auto mapSV2OriginPointCloud = [&](const PointCloudT::Ptr& vSVCloud, PointCloudT::Ptr& vioOriginCloud)
//    {
//        pcl::KdTreeFLANN<PointT> KDTree;
//        KDTree.setInputCloud(vSVCloud);
//        std::uint32_t K = 5;
//        std::vector<int> PointIdxKNNSearch(K);
//        std::vector<float> PointSquaredDistance(K);
//        std::map<std::uint32_t, std::uint32_t> NeighbourLabelCount;
//
//        for (auto& Point : *vioOriginCloud)
//        {
//            PointIdxKNNSearch.clear();
//            PointSquaredDistance.clear();
//            KDTree.nearestKSearch(Point, K, PointIdxKNNSearch, PointSquaredDistance);
//
//            for (auto i : PointIdxKNNSearch)
//            {
//                __checkLabel(NeighbourLabelCount, static_cast<std::uint32_t>(vSVCloud->at(i).a));
//            }
//            Point.a = static_cast<std::uint8_t>(__findMaxCountLabel(NeighbourLabelCount));
//            NeighbourLabelCount.clear();
//        }
//    };
//
//    auto recoverScale2OriginCloud = [&](PointCloudT::Ptr& vioNormalizedCloud,
//        const std::pair<pcl::PointXYZ, pcl::PointXYZ>& vOriginCloudMinMaxCoordinate)
//    {
//        std::pair<pcl::PointXYZ, pcl::PointXYZ> NormalizedCloudMinMaxCoordinate;
//        comMinMax4XYZ(vioNormalizedCloud, NormalizedCloudMinMaxCoordinate);
//        float XLen4NormalizedCloud = NormalizedCloudMinMaxCoordinate.second.x - NormalizedCloudMinMaxCoordinate.first.x;
//        float YLen4NormalizedCloud = NormalizedCloudMinMaxCoordinate.second.y - NormalizedCloudMinMaxCoordinate.first.y;
//        float ZLen4NormalizedCloud = NormalizedCloudMinMaxCoordinate.second.z - NormalizedCloudMinMaxCoordinate.first.z;
//
//        float XLen4OriginCloud = vOriginCloudMinMaxCoordinate.second.x - vOriginCloudMinMaxCoordinate.first.x;
//        float YLen4OriginCloud = vOriginCloudMinMaxCoordinate.second.y - vOriginCloudMinMaxCoordinate.first.y;
//        float ZLen4OriginCLoud = vOriginCloudMinMaxCoordinate.second.z - vOriginCloudMinMaxCoordinate.first.z;
//        for (auto& Point : *vioNormalizedCloud)
//        {
//            float XRelativePosition = (Point.x - NormalizedCloudMinMaxCoordinate.first.x) / XLen4NormalizedCloud;
//            float YRelativePosition = (Point.y - NormalizedCloudMinMaxCoordinate.first.y) / YLen4NormalizedCloud;
//            float ZRelativePosition = (Point.z - NormalizedCloudMinMaxCoordinate.first.z) / ZLen4NormalizedCloud;
//
//            Point.x = vOriginCloudMinMaxCoordinate.first.x + XRelativePosition * XLen4OriginCloud;
//            Point.y = vOriginCloudMinMaxCoordinate.first.y + YRelativePosition * YLen4OriginCloud;
//            Point.z = vOriginCloudMinMaxCoordinate.first.z + ZRelativePosition * ZLen4OriginCLoud;
//  
//        }
//    };
//
//    auto writePointCloud = [&](PointCloudT::Ptr& vCloud,const std::string& vSavePath)
//    {
//        std::ofstream Writer(vSavePath);
//        for (auto& Point : *vCloud)
//        {
//            Writer << Point.x << " ";
//            Writer << Point.y << " ";
//            Writer << Point.z << " ";
//            Writer << static_cast<std::uint32_t>(Point.r) << " ";
//            Writer << static_cast<std::uint32_t>(Point.g) << " ";
//            Writer << static_cast<std::uint32_t>(Point.b) << " ";
//            Writer << static_cast<std::uint32_t>(Point.a) << "\n";
//        }
//        Writer.close();
//    };
//
//    PointCloudT::Ptr SVCloud(new PointCloudT());
//    std::uint32_t vLabelColumnIndex=6;
//    loadSVCloud(TESTDATA_DIR+"MConvergenceResult.txt", SVCloud, vLabelColumnIndex);
//    PointCloudT::Ptr OriginCloud(new PointCloudT());
//    loadCloud(TESTDATA_DIR+"A7GT.txt", OriginCloud);
//    std::pair<pcl::PointXYZ, pcl::PointXYZ> OriginCloudMinMaxCoordinate;
//    comMinMax4XYZ(OriginCloud, OriginCloudMinMaxCoordinate);
//    normalizePointCloud(OriginCloud);
//    mapSV2OriginPointCloud(SVCloud, OriginCloud);
//    recoverScale2OriginCloud(OriginCloud, OriginCloudMinMaxCoordinate);
//    writePointCloud(OriginCloud, TESTDATA_DIR+"ConvergenceResult.txt");
//}

TEST(Exp_confirmUpperBound, computeSegmentationAccuracy)
{
    auto __checkLabel = [](std::map<std::uint32_t, std::uint32_t>& vLabelCount, const std::uint32_t vLabel)
    {
        if (vLabelCount.contains(vLabel))
        {
            vLabelCount.at(vLabel)++;
        }
        else
        {
            vLabelCount.insert(std::make_pair(vLabel, 1));
        }
    };
    auto __findMaxCountLabel = [](const std::map<std::uint32_t, std::uint32_t>& vLabelCount)
    {
        std::pair<std::uint32_t, std::uint32_t> MaxCountLabel(0, 0);
        for (auto& Label : vLabelCount)
        {
            if (Label.second >= MaxCountLabel.second)
                MaxCountLabel = Label;
        }
        return MaxCountLabel.first;
    };
    auto computeAcc = [&](const PointCloudT::Ptr& vPredictedCloud, PointCloudT::Ptr& vGTCloud)
    {
        pcl::KdTreeFLANN<PointT> KDTree;
        KDTree.setInputCloud(vGTCloud);
        std::uint32_t K = 5;
        std::vector<int> PointIdxKNNSearch(K);
        std::vector<float> PointSquaredDistance(K);
        std::map<std::uint32_t, std::uint32_t> NeighbourLabelCount;

        std::uint32_t SameLabelPointCount = 0;
        for (auto& Point : *vPredictedCloud)
        {
            PointIdxKNNSearch.clear();
            PointSquaredDistance.clear();
            KDTree.nearestKSearch(Point, K, PointIdxKNNSearch, PointSquaredDistance);

            for (auto i : PointIdxKNNSearch)
            {
                __checkLabel(NeighbourLabelCount, static_cast<std::uint32_t>(vGTCloud->at(i).a));
            }
            std::uint8_t GTLabel = static_cast<std::uint8_t>(__findMaxCountLabel(NeighbourLabelCount));
            if (GTLabel == Point.a)SameLabelPointCount++;
            NeighbourLabelCount.clear();
        }
        return static_cast<float>(SameLabelPointCount) / vPredictedCloud->size();
    };

    PointCloudT::Ptr PredictedCloud(new PointCloudT());
    loadCloud(TESTDATA_DIR+"MConvergenceResult.txt", PredictedCloud);
    PointCloudT::Ptr GTCloud(new PointCloudT());
    loadCloud(TESTDATA_DIR+"A7GT.txt", GTCloud);
    std::cout << "预测准确率为：" << computeAcc(PredictedCloud, GTCloud) << std::endl;

}