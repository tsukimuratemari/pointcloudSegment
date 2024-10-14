#include "pch.h"
#include "Common.h"
#include "Supervoxel.h"
#include "PostProcessing.h"



class Test_CPostProcessing :public testing::Test
{
protected:
    void SetUp() override
    {

    }

    void TearDown() override
    {

    }
};

TEST() {

}

TEST_F(Test_CPostProcessing, NT_PostProcessing)
{
    PointCloudT::Ptr Cloud(new PointCloudT);
    loadCloud("E:\\PointCloudDataSet\\sum\\orf\\A7.txt", Cloud);

    //extract Supervoxel
    float VoxelResolution = 0.0031f;
    float SeedResolution = 0.009f;
    CSuperVoxel SuperVoxelor(VoxelResolution, SeedResolution);
    SuperVoxelor.setCloud(Cloud);
    std::map <std::uint32_t, pcl::Supervoxel<PointT>::Ptr> Supervoxels;
    SuperVoxelor.extract(Supervoxels);

    std::multimap<std::uint32_t, std::uint32_t> SVAdjacency;
    SuperVoxelor.dumpSupervoxelAdjacency(SVAdjacency);

    std::ifstream Reader("E:\\PointCloudDataSet\\sum\\orf\\ComplicatedResult.txt");
    std::vector<std::vector<double>> SVs;
    std::string Line;
    while (std::getline(Reader, Line))
    {
        std::stringstream SS(Line);
        std::string Item;
        std::vector<double> SV;
        while (std::getline(SS, Item, ' '))
            SV.push_back(std::stod(Item));
        SVs.push_back(SV);
    }
    Reader.close();

    ESemanticCategory Ground = ESemanticCategory::Ground;
    CGroundPostProcessing GroundPostProcessing(Ground, SVs, SVAdjacency);
    GroundPostProcessing.process();
    SVs = GroundPostProcessing.getResult();

    ESemanticCategory Vehicle = ESemanticCategory::Vehicle;
    CVehiclesPostProcessing VehiclesPostProcessing(Vehicle, SVs, SVAdjacency);
    VehiclesPostProcessing.process();
    SVs = VehiclesPostProcessing.getResult();

    ESemanticCategory Building = ESemanticCategory::Building;
    CBuildingPostProcessing BuildingPostProcessing(Building, SVs, SVAdjacency);
    BuildingPostProcessing.process();
    SVs = BuildingPostProcessing.getResult();

    ESemanticCategory Tree = ESemanticCategory::Tree;
    CTreePostProcessing TreePostProcessing(Tree, SVs, SVAdjacency);
    TreePostProcessing.process();
    SVs = TreePostProcessing.getResult();

    std::ofstream Writer("E:\\PointCloudDataSet\\sum\\orf\\PostProcess.txt");
    for (auto& SV : SVs)
    {
        for (auto Item : SV)
        {
            Writer << Item << " ";
        }
        Writer << "\n";
    }
    Writer.close();
}

TEST(mapSegmentation2OriginPointcloud)
{
    auto loadSVCloud = [&](const std::string& vSVCloudFilePath, PointCloudT::Ptr& voSVCloud, std::uint32_t vLabelColumnIndex)
    {
        std::ifstream Reader(vSVCloudFilePath);
        std::string Line;
        while (std::getline(Reader, Line))
        {
            std::stringstream SS(Line);
            std::string TMP;
            PointT Point;
            std::vector<double> String2Double;
            while (std::getline(SS, TMP, ' '))
            {
                String2Double.push_back(std::stod(TMP));
            }

            Point.x = String2Double[0];
            Point.y = String2Double[1];
            Point.z = String2Double[2];
            Point.r = static_cast<std::uint8_t>(String2Double[3]);
            Point.g = static_cast<std::uint8_t>(String2Double[4]);
            Point.b = static_cast<std::uint8_t>(String2Double[5]);
            Point.a = static_cast<std::uint8_t>(String2Double[vLabelColumnIndex]);
            voSVCloud->emplace_back(Point);
        }
        Reader.close();
    };

    auto normalizePointCloud = [&](PointCloudT::Ptr& vioOriginCloud)
    {
        std::pair<pcl::PointXYZ, pcl::PointXYZ> MinMaxCoordinate;
        comMinMax4XYZ(vioOriginCloud, MinMaxCoordinate);

        float XLen = MinMaxCoordinate.second.x - MinMaxCoordinate.first.x;
        float YLen = MinMaxCoordinate.second.y - MinMaxCoordinate.first.y;
        float ZLen = MinMaxCoordinate.second.z - MinMaxCoordinate.first.z;
        float XYScale = XLen / YLen;
        float ZYScale = ZLen / YLen;
        for (std::uint32_t i = 0; i < vioOriginCloud->points.size(); i++)
        {
            vioOriginCloud->points[i].x = (vioOriginCloud->points[i].x - MinMaxCoordinate.first.x) / XLen * XYScale;
            vioOriginCloud->points[i].y = (vioOriginCloud->points[i].y - MinMaxCoordinate.first.y) / YLen;
            vioOriginCloud->points[i].z = (vioOriginCloud->points[i].z - MinMaxCoordinate.first.z) / ZLen * ZYScale;
        }
    };

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

    auto mapSV2OriginPointCloud = [&](const PointCloudT::Ptr& vSVCloud, PointCloudT::Ptr& vioOriginCloud)
    {
        pcl::KdTreeFLANN<PointT> KDTree;
        KDTree.setInputCloud(vSVCloud);
        std::uint32_t K = 5;
        std::vector<int> PointIdxKNNSearch(K);
        std::vector<float> PointSquaredDistance(K);
        std::map<std::uint32_t, std::uint32_t> NeighbourLabelCount;

        for (auto& Point : *vioOriginCloud)
        {
            PointIdxKNNSearch.clear();
            PointSquaredDistance.clear();
            KDTree.nearestKSearch(Point, K, PointIdxKNNSearch, PointSquaredDistance);

            for (auto i : PointIdxKNNSearch)
            {
                __checkLabel(NeighbourLabelCount, static_cast<std::uint32_t>(vSVCloud->at(i).a));
            }
            Point.a = static_cast<std::uint8_t>(__findMaxCountLabel(NeighbourLabelCount));
            NeighbourLabelCount.clear();
        }
    };

    auto recoverScale2OriginCloud = [&](PointCloudT::Ptr& vioNormalizedCloud,
        const std::pair<pcl::PointXYZ, pcl::PointXYZ>& vOriginCloudMinMaxCoordinate)
    {
        std::pair<pcl::PointXYZ, pcl::PointXYZ> NormalizedCloudMinMaxCoordinate;
        comMinMax4XYZ(vioNormalizedCloud, NormalizedCloudMinMaxCoordinate);
        float XLen4NormalizedCloud = NormalizedCloudMinMaxCoordinate.second.x - NormalizedCloudMinMaxCoordinate.first.x;
        float YLen4NormalizedCloud = NormalizedCloudMinMaxCoordinate.second.y - NormalizedCloudMinMaxCoordinate.first.y;
        float ZLen4NormalizedCloud = NormalizedCloudMinMaxCoordinate.second.z - NormalizedCloudMinMaxCoordinate.first.z;

        float XLen4OriginCloud = vOriginCloudMinMaxCoordinate.second.x - vOriginCloudMinMaxCoordinate.first.x;
        float YLen4OriginCloud = vOriginCloudMinMaxCoordinate.second.y - vOriginCloudMinMaxCoordinate.first.y;
        float ZLen4OriginCLoud = vOriginCloudMinMaxCoordinate.second.z - vOriginCloudMinMaxCoordinate.first.z;
        for (auto& Point : *vioNormalizedCloud)
        {
            float XRelativePosition = (Point.x - NormalizedCloudMinMaxCoordinate.first.x) / XLen4NormalizedCloud;
            float YRelativePosition = (Point.y - NormalizedCloudMinMaxCoordinate.first.y) / YLen4NormalizedCloud;
            float ZRelativePosition = (Point.z - NormalizedCloudMinMaxCoordinate.first.z) / ZLen4NormalizedCloud;

            Point.x = vOriginCloudMinMaxCoordinate.first.x + XRelativePosition * XLen4OriginCloud;
            Point.y = vOriginCloudMinMaxCoordinate.first.y + YRelativePosition * YLen4OriginCloud;
            Point.z = vOriginCloudMinMaxCoordinate.first.z + ZRelativePosition * ZLen4OriginCLoud;

        }
    };

    auto writePointCloud = [&](PointCloudT::Ptr& vCloud, const std::string& vSavePath)
    {
        std::ofstream Writer(vSavePath);
        for (auto& Point : *vCloud)
        {
            Writer << Point.x << " ";
            Writer << Point.y << " ";
            Writer << Point.z << " ";
            Writer << static_cast<std::uint32_t>(Point.r) << " ";
            Writer << static_cast<std::uint32_t>(Point.g) << " ";
            Writer << static_cast<std::uint32_t>(Point.b) << " ";
            Writer << static_cast<std::uint32_t>(Point.a) << "\n";
        }
        Writer.close();
    };

    PointCloudT::Ptr SVCloud(new PointCloudT());
    std::uint32_t vLabelColumnIndex = 7;
    loadSVCloud("E:\\PointCloudDataSet\\sum\\orf\\PostProcess.txt", SVCloud, vLabelColumnIndex);
    PointCloudT::Ptr OriginCloud(new PointCloudT());
    loadCloud("E:\\PointCloudDataSet\\sum\\orf\\A7.txt", OriginCloud);
    std::pair<pcl::PointXYZ, pcl::PointXYZ> OriginCloudMinMaxCoordinate;
    comMinMax4XYZ(OriginCloud, OriginCloudMinMaxCoordinate);
    normalizePointCloud(OriginCloud);
    mapSV2OriginPointCloud(SVCloud, OriginCloud);
    recoverScale2OriginCloud(OriginCloud, OriginCloudMinMaxCoordinate);
    writePointCloud(OriginCloud, "E:\\PointCloudDataSet\\sum\\orf\\Result.txt");
}

//TEST(computeSegmentationAccuracy)
//{
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
//    auto computeAcc = [&](const PointCloudT::Ptr& vPredictedCloud, PointCloudT::Ptr& vGTCloud)
//    {
//        pcl::KdTreeFLANN<PointT> KDTree;
//        KDTree.setInputCloud(vGTCloud);
//        std::uint32_t K = 5;
//        std::vector<int> PointIdxKNNSearch(K);
//        std::vector<float> PointSquaredDistance(K);
//        std::map<std::uint32_t, std::uint32_t> NeighbourLabelCount;
//
//        std::uint32_t SameLabelPointCount = 0;
//        for (auto& Point : *vPredictedCloud)
//        {
//            PointIdxKNNSearch.clear();
//            PointSquaredDistance.clear();
//            KDTree.nearestKSearch(Point, K, PointIdxKNNSearch, PointSquaredDistance);
//
//            for (auto i : PointIdxKNNSearch)
//            {
//                __checkLabel(NeighbourLabelCount, static_cast<std::uint32_t>(vGTCloud->at(i).a));
//            }
//            std::uint8_t GTLabel = static_cast<std::uint8_t>(__findMaxCountLabel(NeighbourLabelCount));
//            if (GTLabel == Point.a)SameLabelPointCount++;
//            NeighbourLabelCount.clear();
//        }
//        return static_cast<float>(SameLabelPointCount) / vPredictedCloud->size();
//    };
//
//    PointCloudT::Ptr PredictedCloud(new PointCloudT());
//    loadCloud("C:\\Users\\HPJ\\Desktop\\RFile\\Data\\demo\\A7\\Building\\1\\sumCorrectionResult.txt", PredictedCloud);
//    PointCloudT::Ptr GTCloud(new PointCloudT());
//    loadCloud("C:\\Users\\HPJ\\Desktop\\RFile\\Data\\demo\\A7\\A7GT.txt", GTCloud);
//    std::cout <<"预测准确率为："<< computeAcc(PredictedCloud, GTCloud) << std::endl;
//
//}

//TEST(computeSemanticPercent)
//{
//    PointCloudT::Ptr PredictedCloud(new PointCloudT());
//    loadCloud("C:\\Users\\HPJ\\Desktop\\RFile\\Data\\demo\\A7\\rectifyResult.txt", PredictedCloud);
//    PointCloudT::Ptr OriginCloud(new PointCloudT());
//    loadCloud("C:\\Users\\HPJ\\Desktop\\RFile\\Data\\demo\\A7\\GT.txt", OriginCloud);
//
//    auto computeVehiclePrecent = [&](const PointCloudT::Ptr& vCloud)
//    {
//        std::uint8_t VehicleLabel = 0;
//        std::uint32_t VehiclePointCount = 0;
//        for (auto& Point : *vCloud)
//        {
//            if (Point.a == VehicleLabel)
//                VehiclePointCount++;
//        }
//        return static_cast<float>(VehiclePointCount) / vCloud->size();
//    };
//
//    std::cout << "预测点云中汽车占比：" << computeVehiclePrecent(PredictedCloud) << std::endl;
//    std::cout << "GT中点云占比：" << computeVehiclePrecent(OriginCloud) << std::endl;
//}


//TEST_F(Test_CPostProcessing, NT_MinorTest)
//{
//	Eigen::Vector3d VectorOne(4, 2, 3);
//	Eigen::Vector3d VectorTwo(4, 3, 3);
//    Eigen::Vector3d::Index MaxCol;
//    Eigen::Vector3d::Index MaxRow;
//    VectorOne.maxCoeff(&MaxCol);
//    std::cout << MaxCol << " "<<MaxRow<<std::endl;
//    VectorTwo.maxCoeff(&MaxCol);
//    std::cout << MaxCol << " " << MaxRow << std::endl;
//}

////viusal supervoxel adjacency
//TEST_F(Test_CPostProcessing, NT_visualSupervoxelAdjacency)
//{
//    PointCloudT::Ptr Cloud(new PointCloudT);
//    loadCloud("C:\\Users\\HPJ\\Desktop\\RFile\\Data\\demo\\A7\\AimRegion.txt", Cloud);
//    std::cout << "点云数目" << Cloud->size() << std::endl;
//
//    //extract Supervoxel
//    float VoxelResolution = 0.003f;
//    float SeedResolution = 0.009f;
//    CSuperVoxel SuperVoxelor(VoxelResolution, SeedResolution);
//    SuperVoxelor.setCloud(Cloud);
//    std::map <std::uint32_t, pcl::Supervoxel<PointT>::Ptr> Supervoxels;
//    SuperVoxelor.extract(Supervoxels);
//
//    std::multimap<std::uint32_t, std::uint32_t> SVAdjacency;
//    SuperVoxelor.dumpSupervoxelAdjacency(SVAdjacency);
//
//    std::ofstream Writer("checkSVAdjacency.txt");
//    std::set<std::uint32_t> Neighbourhood;
//
//    auto writeSV2File = [&](PointCloudT::Ptr& vSV,std::uint32_t vLabel) {
//        for (auto PointItr = vSV->begin(); PointItr != vSV->end(); PointItr++)
//        {
//            Writer << PointItr->x << " ";
//            Writer << PointItr->y << " ";
//            Writer << PointItr->z << " ";
//            Writer << vLabel << "\n";
//        }
//    };
//
//    std::uint32_t visualSVSerialNumber = 7643;
//    for (auto& SV : Supervoxels)
//    {
//        if (Neighbourhood.empty())
//        {
//            auto NeighbourhoodItr = SVAdjacency.equal_range(visualSVSerialNumber);
//            for (auto Itr = NeighbourhoodItr.first; Itr != NeighbourhoodItr.second; Itr++)
//                Neighbourhood.insert(Itr->second);  
//        }
//        if(SV.first== visualSVSerialNumber)
//            writeSV2File(SV.second->voxels_, 1);
//        else
//        {
//            if (Neighbourhood.find(SV.first) != Neighbourhood.end())
//                writeSV2File(SV.second->voxels_, 2);
//            else
//                writeSV2File(SV.second->voxels_, 0);
//        }
//    }
//};

//TEST_F(Test_CPostProcessing, NT_genRandomIntegerWithinRange)
//{
//    std::default_random_engine Seed(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
//    std::uniform_int_distribution<> Gen(0, 150);
//
//    for (int i = 0; i < 10;i++)
//    {
//        std::cout << Gen(Seed) << std::endl;
//    }
//}

//TEST_F(Test_CPostProcessing, NT_CountMisjudgeVoxel)
//{
//    PointCloudT::Ptr Cloud(new PointCloudT);
//    std::string CloudPath="C:\\Users\\HPJ\\Desktop\\EXP3\\orgin\\A9.txt";
//    loadCloud(CloudPath, Cloud);
//    std::cout << "点云数目" << Cloud->size() << std::endl;
//
//    //extract Supervoxel
//    float VoxelResolution = 0.005f;
//    float SeedResolution = 0.01f;
//    CSuperVoxel SuperVoxelor(VoxelResolution, SeedResolution);
//    SuperVoxelor.setCloud(Cloud);
//    std::map <std::uint32_t, pcl::Supervoxel<PointT>::Ptr > Supervoxels;
//    SuperVoxelor.extract(Supervoxels);
//    CSemanticCategory SemanticCategoryor;
//    SemanticCategoryor.compute(Cloud, Supervoxels);
//
//    std::string ClassifyResultPath = "C:\\Users\\HPJ\\Desktop\\EXP3\\ClassifyResult\\A9ClassifyResult.txt";
//    std::vector<std::uint32_t> VoxelLabel4Classification;
//    _loadClasifyResult(ClassifyResultPath, VoxelLabel4Classification);
//
//    std::cout << "被误判为Ground的情况:" << std::endl;
//    _countMisjudegSuperVoxel(Supervoxels, VoxelLabel4Classification, ESemanticCategory::Ground);
//    std::cout << std::endl;
//
//    std::cout << "被误判为Building的情况:" << std::endl;
//    _countMisjudegSuperVoxel(Supervoxels, VoxelLabel4Classification, ESemanticCategory::Building);
//    std::cout << std::endl;
//
//    std::cout << "被误判为Tree的情况:" << std::endl;
//    _countMisjudegSuperVoxel(Supervoxels, VoxelLabel4Classification, ESemanticCategory::Tree);
//    std::cout << std::endl;
//
//    std::cout << "被误判为Vegetation的情况:" << std::endl;
//    _countMisjudegSuperVoxel(Supervoxels, VoxelLabel4Classification, ESemanticCategory::Vegetation);
//    std::cout << std::endl;
//
//    std::cout << "被误判为Vehicle的情况:" << std::endl;
//    _countMisjudegSuperVoxel(Supervoxels, VoxelLabel4Classification, ESemanticCategory::Vehicle);
//    std::cout << std::endl;
//}

//TEST_F(Test_CPostProcessing, NT_JudgeVec)
//{
//    PointCloudT::Ptr Cloud(new PointCloudT);
//    std::string CloudPath = "C:\\Users\\HPJ\\Desktop\\EXP3\\TwoClassify\\VecTest.txt";
//    loadCloud(CloudPath, Cloud);
//    std::cout << "点云数目" << Cloud->size() << std::endl;
//
//    //extract Supervoxel
//    float VoxelResolution = 0.008f;
//    float SeedResolution = 0.01f;
//    CSuperVoxel SuperVoxelor(VoxelResolution, SeedResolution);
//    SuperVoxelor.setCloud(Cloud);
//    std::map <std::uint32_t, pcl::Supervoxel<PointT>::Ptr > Supervoxels;
//    SuperVoxelor.extract(Supervoxels);
//    CSemanticCategory SemanticCategoryor;
//    SemanticCategoryor.compute(Cloud, Supervoxels);
//
//    std::string ClassifyResultPath = "C:\\Users\\HPJ\\Desktop\\EXP3\\TwoClassify\\VecClassifyResult.txt";
//    std::vector<std::uint32_t> VoxelLabel4Classification;
//    _loadClasifyResult(ClassifyResultPath, VoxelLabel4Classification);
//    _comAverageGroundHeight(Supervoxels, VoxelLabel4Classification);
//}


//class TEST_ConvertRGB2LAB :public testing::Test
//{
//protected:
//    void SetUp() override
//    {
//
//    }
//
//    void TearDown() override
//    {
//
//    }
//
//    void _genSupervoxelCloud(const std::map<std::uint32_t, pcl::Supervoxel<PointT>::Ptr>& vSupervoxels, PointCloudT::Ptr& voSupervoxelCloud)
//    {
//        voSupervoxelCloud->clear();
//        for (auto SVItr = vSupervoxels.cbegin(); SVItr != vSupervoxels.cend(); SVItr++)
//        {
//            *voSupervoxelCloud += *SVItr->second->voxels_;
//        }
//    }
//    void _loadCloud(const std::string& vCloudTXTFilePath, PointCloudT::Ptr& voCloud)
//    {
//        std::ifstream Reader(vCloudTXTFilePath);
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
//            Point.r = static_cast<std::uint32_t>(String2Double[3]);
//            Point.g = static_cast<std::uint32_t>(String2Double[4]);
//            Point.b = static_cast<std::uint32_t>(String2Double[5]);
//			Point.a = static_cast<std::uint32_t>(String2Double[6]);
//            voCloud->push_back(Point);
//        }
//        Reader.close();
//    }
//
//    void _labelSupervoxel(std::map <std::uint32_t, pcl::Supervoxel<PointT>::Ptr >& voSupervoxels, const std::pair<pcl::PointXYZ, pcl::PointXYZ>& vVoxelCloudMinMaxCoordinate, const PointCloudT::Ptr& vOriginCloud)
//    {
//        float XLen4NormalizedCloud = vVoxelCloudMinMaxCoordinate.second.x - vVoxelCloudMinMaxCoordinate.first.x;
//        float YLen4NormalizedCloud = vVoxelCloudMinMaxCoordinate.second.y - vVoxelCloudMinMaxCoordinate.first.y;
//        float ZLen4NormalizedCloud = vVoxelCloudMinMaxCoordinate.second.z - vVoxelCloudMinMaxCoordinate.first.z;
//
//        std::pair<pcl::PointXYZ, pcl::PointXYZ> OriginCloudMinMaxCoordinate;
//        comMinMax4XYZ(vOriginCloud, OriginCloudMinMaxCoordinate);
//        float XLen4OriginCloud = OriginCloudMinMaxCoordinate.second.x - OriginCloudMinMaxCoordinate.first.x;
//        float YLen4OriginCloud = OriginCloudMinMaxCoordinate.second.y - OriginCloudMinMaxCoordinate.first.y;
//        float ZLen4OriginCLoud = OriginCloudMinMaxCoordinate.second.z - OriginCloudMinMaxCoordinate.first.z;
//
//        pcl::KdTreeFLANN<PointT> KDTree;
//        KDTree.setInputCloud(vOriginCloud);
//        std::uint32_t K = 5;
//        std::vector<int> PointIdxKNNSearch(K);
//        std::vector<float> PointSquaredDistance(K);
//
//        auto findMaxCountLabel = [](const std::map<std::uint32_t, std::uint32_t>& vLabelCount) {
//            std::pair<std::uint32_t, std::uint32_t> MaxCountLabel(0, 0);
//            for (auto& Label : vLabelCount)
//            {
//                if (Label.second >= MaxCountLabel.second)
//                    MaxCountLabel = Label;
//            }
//            return MaxCountLabel;
//        };
//
//        auto checkLabel = [](std::map<std::uint32_t, std::uint32_t>& vLabelCount, std::uint32_t vLabel)
//        {
//            if (vLabelCount.find(vLabel) != vLabelCount.end())
//            {
//                vLabelCount.at(vLabel)++;
//            }
//            else
//            {
//                vLabelCount.insert(std::make_pair(vLabel, 1));
//            }
//        };
//
//        for (auto& SV : voSupervoxels)
//        {
//            std::map<std::uint32_t, std::uint32_t> PointLabelCount;
//            for (auto& Point : SV.second->voxels_->points)
//            {
//                float XRelativePosition = (Point.x - vVoxelCloudMinMaxCoordinate.first.x) / XLen4NormalizedCloud;
//                float YRelativePosition = (Point.y - vVoxelCloudMinMaxCoordinate.first.y) / YLen4NormalizedCloud;
//                float ZRelativePosition = (Point.z - vVoxelCloudMinMaxCoordinate.first.z) / ZLen4NormalizedCloud;
//
//                float XPosInOriginCloud = OriginCloudMinMaxCoordinate.first.x + XRelativePosition * XLen4OriginCloud;
//                float YPosInOriginCloud = OriginCloudMinMaxCoordinate.first.y + YRelativePosition * YLen4OriginCloud;
//                float ZPosInOriginCloud = OriginCloudMinMaxCoordinate.first.z + ZRelativePosition * ZLen4OriginCLoud;
//                PointT TMPPoint(XPosInOriginCloud, YPosInOriginCloud, ZPosInOriginCloud);
//                PointIdxKNNSearch.clear();
//                PointSquaredDistance.clear();
//                KDTree.nearestKSearch(TMPPoint, K, PointIdxKNNSearch, PointSquaredDistance);
//
//                std::map<std::uint32_t, std::uint32_t> NeighbourLabelCount;
//                for (auto i : PointIdxKNNSearch)
//                {
//                    checkLabel(NeighbourLabelCount, static_cast<std::uint32_t>(vOriginCloud->points[i].a));
//                }
//                Point.a = static_cast<std::uint8_t>(findMaxCountLabel(NeighbourLabelCount).first);
//                checkLabel(PointLabelCount, static_cast<std::uint32_t>(Point.a));
//                NeighbourLabelCount.clear();
//            }
//            SV.second->centroid_.a = static_cast<std::uint8_t>(findMaxCountLabel(PointLabelCount).first);
//        }
//    }
//
//    void _writeSupervoxelWithCentroidLabel2TXT(const std::map<std::uint32_t, std::shared_ptr<pcl::Supervoxel<PointT>>>& vSupervoxels, const std::string& vOutputFileName)
//    {
//        std::ofstream Writer(vOutputFileName);
//        for (auto& SV : vSupervoxels)
//        {
//            for (auto& Point : SV.second->voxels_->points)
//            {
//                Writer << Point.x << " ";
//                Writer << Point.y << " ";
//                Writer << Point.z << " ";
//                Writer << static_cast<std::uint32_t>(Point.r) << " ";
//                Writer << static_cast<std::uint32_t>(Point.g) << " ";
//                Writer << static_cast<std::uint32_t>(Point.b) << " ";
//                Writer << static_cast<std::uint32_t>(SV.second->centroid_.a) << " ";
//                Writer << SV.first << "\n";
//            }
//        }
//        Writer.close();
//    }
//
//    void _writeSupervoxelWithPointLabel2TXT(const std::map<std::uint32_t, std::shared_ptr<pcl::Supervoxel<PointT>>>& vSupervoxels, const std::string& vOutputFileName)
//    {
//        std::ofstream Writer(vOutputFileName);
//        for (auto& SV : vSupervoxels)
//        {
//            for (auto& Point : SV.second->voxels_->points)
//            {
//                Writer << Point.x << " ";
//                Writer << Point.y << " ";
//                Writer << Point.z << " ";
//                Writer << static_cast<std::uint32_t>(Point.r) << " ";
//                Writer << static_cast<std::uint32_t>(Point.g) << " ";
//                Writer << static_cast<std::uint32_t>(Point.b) << " ";
//                Writer << static_cast<std::uint32_t>(Point.a) << " ";
//                Writer << SV.first << "\n";
//            }
//        }
//        Writer.close();
//    }
//};
//
//class Test_PFH :public testing::Test {
//protected:
//    void SetUp() override
//    {
//
//    }
//
//    void TearDown() override
//    {
//
//    }
//
//    void _loadCloud(const std::string& vCloudTXTFilePath, PointCloudT::Ptr& voCloud)
//    {
//        std::ifstream Reader(vCloudTXTFilePath);
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
//            Point.r = static_cast<std::uint32_t>(String2Double[3]);
//            Point.g = static_cast<std::uint32_t>(String2Double[4]);
//            Point.b = static_cast<std::uint32_t>(String2Double[5]);
//            Point.a = static_cast<std::uint32_t>(String2Double[6]);
//            voCloud->push_back(Point);
//        }
//        Reader.close();
//    }
//};

//TEST_F(Test_PFH, TestName)
//{
//    PointCloudT::Ptr Cloud(new PointCloudT);
//    _loadCloud("C:\\Users\\HPJ\\Desktop\\SCU\\OldNewPreTest.txt", Cloud);
//    std::cout << "点云数目" << Cloud->size() << std::endl;
//
//    float VoxelResolution = 0.005f;
//    float SeedResolution = 0.01f;
//    CSuperVoxel SuperVoxelor(VoxelResolution, SeedResolution);
//    SuperVoxelor.setCloud(Cloud);
//    std::map <std::uint32_t, pcl::Supervoxel<PointT>::Ptr > Supervoxels;
//    SuperVoxelor.extract(Supervoxels);
//
//    //compute SemanticCategory
//    CSemanticCategory SemanticCategoryor;
//    SemanticCategoryor.compute(Cloud, Supervoxels);
//
//    float K = 0.005;
//    std::vector<int> PointIdxKNNSearch;
//    std::vector<float> PointSquaredDistance;
//    //normal estimation 
//	pcl::NormalEstimation<PointT, pcl::Normal> NormalEstimation;
//	NormalEstimation.setInputCloud(Cloud);
//	pcl::search::KdTree<PointT>::Ptr Tree(new pcl::search::KdTree<PointT>());
//	NormalEstimation.setSearchMethod(Tree);
//	pcl::PointCloud<pcl::Normal>::Ptr CloudNormals(new pcl::PointCloud<pcl::Normal>());
//	NormalEstimation.setRadiusSearch(K);
//	NormalEstimation.compute(*CloudNormals);
//
//    pcl::PFHEstimation<PointT, pcl::Normal, pcl::PFHSignature125> PFHDescritor;
//    std::multimap<std::uint32_t, Eigen::VectorXf> PFHs;
//    std::uint32_t NRSplit = 2;
//    std::uint32_t PFHDimension=std::pow(NRSplit,3);
//    Eigen::VectorXf PFH(PFHDimension);
//    for (auto& SV : Supervoxels)
//    {
//        Tree->radiusSearch(Supervoxels.cbegin()->second->centroid_, 0.01, PointIdxKNNSearch, PointSquaredDistance);
//        PFHDescritor.computePointPFHSignature(*Cloud, *CloudNormals, PointIdxKNNSearch, NRSplit, PFH);
//        PFHs.insert(std::make_pair(static_cast<std::uint32_t>(SV.second->centroid_.a), PFH));
//    }
//
//    for (int i = 1; i < 6; i++)
//    {
//        Eigen::VectorXf SumPFH = Eigen::VectorXf::Zero(PFHDimension);
//        std::uint32_t Num = 0;
//        for (auto Itr = PFHs.equal_range(i).first; Itr != PFHs.equal_range(i).second; Itr++)
//        {
//            SumPFH += Itr->second;
//            Num++;
//        }
//        SumPFH /= Num;
//
//        for (int i = 0; i < SumPFH.size(); i++)
//        {
//            std::cout << SumPFH[i] << " ";
//        }
//        std::cout<<std::endl;
//    }
//
//}

//TEST_F(TEST_ConvertRGB2LAB, TestName) {
//
//    PointCloudT::Ptr Cloud(new PointCloudT);
//    _loadCloud("C:\\Users\\HPJ\\Desktop\\SCU\\OldNewPreTest.txt", Cloud);
//    std::cout << "点云数目" << Cloud->size() << std::endl;
//
//    //extract Supervoxel
//    float VoxelResolution = 0.003f;
//    float SeedResolution = 0.01f;
//    CSuperVoxel SuperVoxelor(VoxelResolution, SeedResolution);
//    SuperVoxelor.setCloud(Cloud);
//    std::map <std::uint32_t, pcl::Supervoxel<PointT>::Ptr > Supervoxels;
//    SuperVoxelor.extract(Supervoxels);
//
//    PointCloudT::Ptr SupervoxelCloud(new PointCloudT());
//    _genSupervoxelCloud(Supervoxels, SupervoxelCloud);
//
//    std::pair<pcl::PointXYZ, pcl::PointXYZ> VoxelCloudMinMaxCoordinate;
//    comMinMax4XYZ(SupervoxelCloud, VoxelCloudMinMaxCoordinate);
//    
//    _labelSupervoxel(Supervoxels, VoxelCloudMinMaxCoordinate, Cloud);
//    _writeSupervoxelWithCentroidLabel2TXT(Supervoxels, "SupervoxelWithCentroidLabel.txt");
//    _writeSupervoxelWithPointLabel2TXT(Supervoxels, "SupervoxelWithPointLabel.txt");
//}

//TEST_F(TEST_ConvertRGB2LAB, TestName) {
//    for (int i = 0; i < 10; i++)
//    {
//        std::string NowTime = std::to_string(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
//        std::cout << NowTime << std::endl;
//    }
//}