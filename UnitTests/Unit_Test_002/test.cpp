#include "pch.h"
#include "data.h"
#include "Utils.h"
#include "experimenter.h"
#include "online_rf.h"

float computeAcc(const PointCloudT::Ptr& vPredictedCloud, PointCloudT::Ptr& vGTCloud);
void loadCloud(const std::string& vCloudTXTFilePath, PointCloudT::Ptr& voCloud);

const std::string TESTDATA_DIR = "C:\\Users\\HPJ\\Desktop\\RFile\\Data\\demo\\A7\\001-002\\A7GT\\";

TEST(Exp1_inheritInitialSegmentationResult, getBinaryClassificationResults) {
    CPointCloudTool PointCloudTool;
    vector<vector<double>> InitialSegmentationResult;
    string InitialSegmentationResultFilePath = "C:\\Users\\HPJ\\Desktop\\RFile\\Data\\demo\\A7\\SVofRegion.txt";
    PointCloudTool.loadOriginArray(InitialSegmentationResultFilePath, InitialSegmentationResult);
    std::string ConfFilePath = "..\\..\\OnlineLearning\\conf\\omcb.conf";
    Hyperparameters HP(ConfFilePath);
    DataSet TestData(InitialSegmentationResult);

    auto getBinaryClassificationConvergenceResult = [&](vector<vector<double>> vInitalResult, ESemanticCategory vSemanticLabel,std::string vSaveFileName)
    {
        PointCloudTool.binarizeDLTrainData(vSemanticLabel, vInitalResult);
        DataSet TrainData(vInitalResult);

        Classifier* Model = new OnlineRF(HP, TrainData.m_numClasses, TrainData.m_numFeatures, TrainData.m_minFeatRange, TrainData.m_maxFeatRange);
        train(Model, TrainData, HP);
        vector<Result> Results = test(Model, TestData, HP);
        PointCloudTool.visualizeORFApproachDLResult(vInitalResult, Results);
        PointCloudTool.saveArrayFile("C:\\Users\\HPJ\\Desktop\\Paper\\Exp1\\"+vSaveFileName, vInitalResult);
        delete Model;
    };

    getBinaryClassificationConvergenceResult(InitialSegmentationResult, ESemanticCategory::Building, "BuildingConvergenceResult.txt");
    getBinaryClassificationConvergenceResult(InitialSegmentationResult, ESemanticCategory::Vehicle, "VehicleConvergenceResult.txt");
    getBinaryClassificationConvergenceResult(InitialSegmentationResult, ESemanticCategory::Ground, "GroundConvergenceResult.txt");
    getBinaryClassificationConvergenceResult(InitialSegmentationResult, ESemanticCategory::Tree, "TreeConvergenceResult.txt");
}

TEST(Exp1_inheritInitialSegmentationResult, computeBinaryClassificationInheritanceAccaury)
{
    CPointCloudTool PointCloudOperator;
    std::vector<vector<double>> InitialSegmentationResult;
    std::string InitialResultPath = TESTDATA_DIR+"SVofRegion.txt";
    std::uint32_t LabelCloumnIndex = 26;
    PointCloudOperator.setLabelColumnIndex(LabelCloumnIndex);
    PointCloudOperator.loadArrayFile(InitialResultPath, InitialSegmentationResult);

    auto __convertDataFormat2Pointcloud = [&](const std::vector<vector<double>> vPointcloud, PointCloudT::Ptr& voPointcloud)
    {
        for (auto& Point : vPointcloud)
            voPointcloud->emplace_back(Point[0], Point[1], Point[2], Point[3], Point[4], Point[5], Point[LabelCloumnIndex]);
    };

    auto __computeBinaryClassificationConvergenceAcc = [&](const ESemanticCategory& vPositiveSemanticLabel,
        std::vector<vector<double>> vMultiClassInitialSegmentationResult,const std::string& vConvergenceResultPath)
    {
        PointCloudOperator.binarizeDLTrainData(vPositiveSemanticLabel, vMultiClassInitialSegmentationResult);
        PointCloudT::Ptr GroundTruth(new PointCloudT);
        __convertDataFormat2Pointcloud(vMultiClassInitialSegmentationResult, GroundTruth);
        PointCloudT::Ptr ConvergenceResult(new PointCloudT);
        loadCloud(vConvergenceResultPath, ConvergenceResult);
        return computeAcc(GroundTruth,ConvergenceResult);
    };

	ESemanticCategory CurrentPositiveSemanticLabel(ESemanticCategory::Building);
	std::string ConvergenceResultPath = TESTDATA_DIR+"BuildingConvergenceResult.txt";
	std::cout << "Building逼近准确率为：" <<__computeBinaryClassificationConvergenceAcc(CurrentPositiveSemanticLabel, InitialSegmentationResult, ConvergenceResultPath) << std::endl;

	CurrentPositiveSemanticLabel = ESemanticCategory::Tree;
	ConvergenceResultPath = TESTDATA_DIR+"TreeConvergenceResult.txt";
	std::cout << "Tree逼近准确率为：" <<__computeBinaryClassificationConvergenceAcc(CurrentPositiveSemanticLabel, InitialSegmentationResult, ConvergenceResultPath) << std::endl;

	CurrentPositiveSemanticLabel = ESemanticCategory::Ground;
	ConvergenceResultPath = TESTDATA_DIR+"GroundConvergenceResult.txt";
	std::cout << "Ground逼近准确率为：" <<__computeBinaryClassificationConvergenceAcc(CurrentPositiveSemanticLabel, InitialSegmentationResult, ConvergenceResultPath) << std::endl;

	CurrentPositiveSemanticLabel = ESemanticCategory::Vehicle;
	ConvergenceResultPath = TESTDATA_DIR+"VehicleConvergenceResult.txt";
	std::cout << "Vehicle逼近准确率为：" <<__computeBinaryClassificationConvergenceAcc(CurrentPositiveSemanticLabel, InitialSegmentationResult, ConvergenceResultPath) << std::endl;
}

//TEST(Exp1_inheritInitialSegmentationResult, computeWholeSceneInheritanceAccaury)
//{
//    auto __loadBCResult = [](const std::string& vSVCloudFilePath, ESemanticCategory& vSemanticLabel,std::unordered_multimap<std::uint32_t, ESemanticCategory>& voBCResults)
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
//            ESemanticCategory Label = static_cast<EBiClassificationSemanticCategory>(SingleRowData[26]) == EBiClassificationSemanticCategory::PositiveCategory ? vSemanticLabel : ESemanticCategory::Undefined;
//            voBCResults.emplace(static_cast<std::uint32_t>(SingleRowData[25]), Label);
//        }
//        Reader.close();
//    };
//    auto __combineBCResults = [](std::unordered_multimap<std::uint32_t, ESemanticCategory>& vBCResults,std::unordered_map<std::uint32_t, ESemanticCategory>& voMCResult)
//    {
//        for (auto Itr = vBCResults.cbegin(); Itr!=vBCResults.cend();++Itr)
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
//    auto __loadCloudAndStoreMCResult = [&](const std::string& vCloudFilePath, const std::string& vSaveFilePath,std::unordered_map<std::uint32_t, ESemanticCategory>& vMCResult)
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
//            Writer << static_cast<double>(vMCResult.at(SingleRowData[25]))<<"\n";
//        }
//        Reader.close();
//        Writer.close();
//    };
//
//    std::string BCResultFilePath = "C:\\Users\\HPJ\\Desktop\\Paper\\Exp1\\SV\\BuildingConvergenceResult.txt";
//    ESemanticCategory BCPositiveLabel = ESemanticCategory::Building;
//    std::unordered_multimap<std::uint32_t, ESemanticCategory> BCResults;
//    __loadBCResult(BCResultFilePath, BCPositiveLabel, BCResults);
//
//    BCResultFilePath= "C:\\Users\\HPJ\\Desktop\\Paper\\Exp1\\SV\\TreeConvergenceResult.txt";
//    BCPositiveLabel = ESemanticCategory::Tree;
//    __loadBCResult(BCResultFilePath, BCPositiveLabel, BCResults);
//
//    BCResultFilePath = "C:\\Users\\HPJ\\Desktop\\Paper\\Exp1\\SV\\GroundConvergenceResult.txt";
//    BCPositiveLabel = ESemanticCategory::Ground;
//    __loadBCResult(BCResultFilePath, BCPositiveLabel, BCResults);
//
//    BCResultFilePath = "C:\\Users\\HPJ\\Desktop\\Paper\\Exp1\\SV\\VehicleConvergenceResult.txt";
//    BCPositiveLabel = ESemanticCategory::Vehicle;
//    __loadBCResult(BCResultFilePath, BCPositiveLabel, BCResults);
//
//    std::unordered_map<std::uint32_t, ESemanticCategory> MCResult;
//    __combineBCResults(BCResults, MCResult);
//
//    const std::string SVCloudFilePath = BCResultFilePath;
//    const std::string SaveFilePath= "C:\\Users\\HPJ\\Desktop\\Paper\\Exp1\\SV\\MConvergenceResult.txt";
//    __loadCloudAndStoreMCResult(SVCloudFilePath, SaveFilePath, MCResult);
//}

float computeAcc(const PointCloudT::Ptr& vPredictedCloud, PointCloudT::Ptr& vGTCloud)
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
}
//void loadCloud(const std::string& vCloudTXTFilePath, PointCloudT::Ptr& voCloud)
//{
//    std::ifstream Reader(vCloudTXTFilePath);
//    std::string Line;
//    while (std::getline(Reader, Line))
//    {
//        std::stringstream SS(Line);
//        std::string TMP;
//        PointT Point;
//        std::vector<double> String2Double;
//        while (std::getline(SS, TMP, ' '))
//        {
//            String2Double.push_back(std::stod(TMP));
//        }
//
//        Point.x = String2Double[0];
//        Point.y = String2Double[1];
//        Point.z = String2Double[2];
//        Point.r = static_cast<std::uint8_t>(String2Double[3]);
//        Point.g = static_cast<std::uint8_t>(String2Double[4]);
//        Point.b = static_cast<std::uint8_t>(String2Double[5]);
//        Point.a = static_cast<std::uint8_t>(String2Double[26]);
//        voCloud->push_back(Point);
//    }
//    Reader.close();
//}

