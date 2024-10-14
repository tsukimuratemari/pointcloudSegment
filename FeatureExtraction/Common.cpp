#include"pch.h"
#include"Common.h"

void comMinMax4XYZ(const PointCloudT::Ptr& vCloud, std::pair<pcl::PointXYZ, pcl::PointXYZ>& vMinMaxCoordinate)
{
    float XMin = FLT_MAX, XMax = FLT_MIN;
    float YMin = FLT_MAX, YMax = FLT_MIN;
    float ZMin = FLT_MAX, ZMax = FLT_MIN;

    for (std::uint32_t i = 0; i < vCloud->points.size(); i++)
    {
        if (vCloud->points[i].x < XMin)XMin = vCloud->points[i].x;
        if (vCloud->points[i].x > XMax)XMax = vCloud->points[i].x;
        if (vCloud->points[i].y < YMin)YMin = vCloud->points[i].y;
        if (vCloud->points[i].y > YMax)YMax = vCloud->points[i].y;
        if (vCloud->points[i].z < ZMin)ZMin = vCloud->points[i].z;
        if (vCloud->points[i].z > ZMax)ZMax = vCloud->points[i].z;
    }

    vMinMaxCoordinate.first.x = XMin;
    vMinMaxCoordinate.first.y = YMin;
    vMinMaxCoordinate.first.z = ZMin;
    vMinMaxCoordinate.second.x = XMax;
    vMinMaxCoordinate.second.y = YMax;
    vMinMaxCoordinate.second.z = ZMax;
}

void computeAccByMapPredictedCloud2Groundtruth(const PointCloudT::Ptr& vPredictedCloud, const PointCloudT::Ptr& vGroundTruthCloud)
{
    pcl::search::KdTree<PointT>::Ptr KdTree4CroundTruthCloud;
    auto __initializeKdTree = [&]()
    {
        _HIVE_EARLY_RETURN(vGroundTruthCloud && vGroundTruthCloud->empty(), "Point cloud need to be initialized with non-empty cloud!", false);
        if (KdTree4CroundTruthCloud && KdTree4CroundTruthCloud->getInputCloud() == vGroundTruthCloud)return true;
        KdTree4CroundTruthCloud = std::make_shared<pcl::search::KdTree<PointT>>();
        KdTree4CroundTruthCloud->setInputCloud(vGroundTruthCloud);
        return true;
    };

    auto __searchNeighbours = [&](const PointT& vPoint, std::vector<int>& voNeighbourPointIdxs)
    {
        _HIVE_EARLY_RETURN(KdTree4CroundTruthCloud && KdTree4CroundTruthCloud->getInputCloud()->empty(), "KDTree isn't be initialized!", 0);
        voNeighbourPointIdxs.clear();
        std::vector<float> TempVector;
        int NeighbourNum = 1;
        KdTree4CroundTruthCloud->nearestKSearch(vPoint, NeighbourNum, voNeighbourPointIdxs, TempVector);
        return voNeighbourPointIdxs.at(0);
    };

    __initializeKdTree();
    int RightCount = 0;
    std::vector<int> NeighbourPointIdxs;
    for (auto& Point : vPredictedCloud->points)
    {
        if ((int)Point.a == (int)(vGroundTruthCloud->points.at(__searchNeighbours(Point, NeighbourPointIdxs))).a)
        {
            RightCount++;
        }
    }
    std::cout << "Accarucy:" << (float)RightCount / vPredictedCloud->size() << std::endl;
}

void genSupervoxelCloud(const std::map<std::uint32_t, pcl::Supervoxel<PointT>::Ptr>& vSupervoxels, PointCloudT& voSupervoxelCloud)
{
    voSupervoxelCloud.clear();
    for (auto SVItr = vSupervoxels.cbegin(); SVItr != vSupervoxels.cend(); SVItr++)
    {
        voSupervoxelCloud += *SVItr->second->voxels_;
    }
}

void writeFeature2LIBSVM(const std::map<std::uint32_t, SSupervoxelVFeatures>& vSVFeatureList, const int vSemanticCategoryNum, const std::string& vOutputFileName)
{
    std::ofstream Writer(vOutputFileName);
    uint32_t LabelColumnIndexInLIBSVM = 1;
    uint32_t SVFeatureNum = 19;
    Writer << vSVFeatureList.size() << " " << SVFeatureNum << " " << vSemanticCategoryNum << " " << LabelColumnIndexInLIBSVM << std::endl;

    for (auto& SVFeature : vSVFeatureList)
    {
        auto FeatureBasedEigenvalue = SVFeature.second._FeaturesBasedEigenvalue.getFeatures();
        Writer << SVFeature.second._SemanticCategory << " ";
        for (std::uint32_t i = 0; i < FeatureBasedEigenvalue.size(); i++)
        {
            Writer << i + 1 << ":" << FeatureBasedEigenvalue[i] << " ";
        }
        for (std::uint32_t i = 0; i < SVFeature.second._PFH.size(); i++)
        {
            Writer << FeatureBasedEigenvalue.size() + i + 1 << ":" << SVFeature.second._PFH[i] << " ";
        }
        Writer << FeatureBasedEigenvalue.size() + SVFeature.second._PFH.size() + 1 << ":" << SVFeature.second._LAB[0] << " ";
        Writer << FeatureBasedEigenvalue.size() + SVFeature.second._PFH.size() + 2 << ":" << SVFeature.second._LAB[1] << " ";
        Writer << FeatureBasedEigenvalue.size() + SVFeature.second._PFH.size() + 3 << ":" << SVFeature.second._LAB[2] << " ";
        Writer << FeatureBasedEigenvalue.size() + SVFeature.second._PFH.size() + 4 << ":" << SVFeature.second._Direction << " ";
        Writer << FeatureBasedEigenvalue.size() + SVFeature.second._PFH.size() + 5 << ":" << SVFeature.second._LocalDensity << " ";
        Writer << FeatureBasedEigenvalue.size() + SVFeature.second._PFH.size() + 6 << ":" << SVFeature.second._RelativeElevation << "\n";
        
    }
    Writer.close();
}

void writeFeature2TXT(const std::map<std::uint32_t, SSupervoxelVFeatures>& vSVFeatureList, const std::string& vOutputFileName)
{
    std::ofstream Writer(vOutputFileName);
    for (auto& SVFeature : vSVFeatureList)
    {
        auto FeatureBasedEigenvalue = SVFeature.second._FeaturesBasedEigenvalue.getFeatures();
        for (std::uint32_t i = 0; i < FeatureBasedEigenvalue.size(); i++)
        {
            Writer << FeatureBasedEigenvalue[i]<<" ";
        }
        for (std::uint32_t i = 0; i < SVFeature.second._PFH.size(); i++)
        {
            Writer << SVFeature.second._PFH[i] << " ";
        }
        Writer << SVFeature.second._LAB[0] << " ";
        Writer << SVFeature.second._LAB[1] << " ";
        Writer << SVFeature.second._LAB[2] << " ";
        Writer << SVFeature.second._Direction << " ";
        Writer << SVFeature.second._LocalDensity << " ";
        Writer << SVFeature.second._RelativeElevation << " ";
        Writer << SVFeature.second._SemanticCategory << "\n";
    }
    Writer.close();
}

void writeFeatureWithPoint2TXT(const std::map <std::uint32_t, pcl::Supervoxel<PointT>::Ptr >& vSupervoxels, const std::map<std::uint32_t, SSupervoxelVFeatures>& vSVFeatureList, const std::string& vOutputFileName)
{
    std::ofstream Writer(vOutputFileName);
    for (auto& SV : vSupervoxels)
    {
        auto SVFeature = vSVFeatureList.at(SV.first);
        auto FeatureBasedEigenvalue = SVFeature._FeaturesBasedEigenvalue.getFeatures();
        for (auto& Point : SV.second->voxels_->points)
        {
            Writer << Point.x << " ";
            Writer << Point.y << " ";
            Writer << Point.z << " ";
            Writer << static_cast<std::uint32_t>(Point.r) << " ";
            Writer << static_cast<std::uint32_t>(Point.g) << " ";
            Writer << static_cast<std::uint32_t>(Point.b) << " ";
            for (std::uint32_t i = 0; i < FeatureBasedEigenvalue.size(); i++)
            {
                Writer << FeatureBasedEigenvalue[i] << " ";
            }
            for (std::uint32_t i = 0; i < SVFeature._PFH.size(); i++)
            {
                Writer << SVFeature._PFH[i] << " ";
            }
            Writer << SVFeature._LAB[0] << " ";
            Writer << SVFeature._LAB[1] << " ";
            Writer << SVFeature._LAB[2] << " ";
            Writer << SVFeature._Direction << " ";
            Writer << SVFeature._LocalDensity << " ";
            Writer << SVFeature._RelativeElevation << " ";
            Writer << SV.first << " ";
            Writer << SVFeature._SemanticCategory << "\n";
        }
    }
    Writer.close();
}

void writeSupervoxelWithPointLabel2TXT(const std::map<std::uint32_t, std::shared_ptr<pcl::Supervoxel<PointT>>>& vSupervoxels, const std::string& vOutputFileName)
{
    std::ofstream Writer(vOutputFileName);
    for (auto& SV : vSupervoxels)
    {
        for (auto& Point : SV.second->voxels_->points)
        {
            Writer << Point.x << " ";
            Writer << Point.y << " ";
            Writer << Point.z << " ";
            Writer << static_cast<std::uint32_t>(Point.r) << " ";
            Writer << static_cast<std::uint32_t>(Point.g) << " ";
            Writer << static_cast<std::uint32_t>(Point.b) << " ";
            Writer << static_cast<std::uint32_t>(Point.a) << " ";
            Writer << SV.first << "\n";
        }
    }
    Writer.close();
}

void writeSupervoxelWithCentroidLabel2TXT(const std::map<std::uint32_t, std::shared_ptr<pcl::Supervoxel<PointT>>>& vSupervoxels, const std::string& vOutputFileName)
{
    std::ofstream Writer(vOutputFileName);
    for (auto& SV : vSupervoxels)
    {
        for (auto& Point : SV.second->voxels_->points)
        {
            Writer << Point.x << " ";
            Writer << Point.y << " ";
            Writer << Point.z << " ";
            Writer << static_cast<std::uint32_t>(Point.r) << " ";
            Writer << static_cast<std::uint32_t>(Point.g) << " ";
            Writer << static_cast<std::uint32_t>(Point.b) << " ";
            Writer << static_cast<std::uint32_t>(SV.second->centroid_.a) << " ";
            Writer << SV.first << "\n";
        }
    }
    Writer.close();
}

void loadCloud(const std::string& vCloudTXTFilePath, PointCloudT::Ptr& voCloud)
{
    std::ifstream Reader(vCloudTXTFilePath);
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
        Point.r = static_cast<int>(String2Double[3]);
        Point.g = static_cast<int>(String2Double[4]);
        Point.b = static_cast<int>(String2Double[5]);
        Point.a = static_cast<int>(String2Double[6]);
        voCloud->push_back(Point);
    }
    Reader.close();
}

void loadClassifyResultAndComputeAcc(const std::string& vClassifyResultTXTPath, std::map<std::uint32_t, std::shared_ptr<pcl::Supervoxel<PointT>>> vSupervoxels)
{
    std::vector<std::uint32_t> VoxelLabel;
    std::ifstream AnthoerReader(vClassifyResultTXTPath);
    std::string AnthoerLine;
    while (std::getline(AnthoerReader, AnthoerLine))
    {
        std::stringstream SSS(AnthoerLine);
        std::string TMP;
        std::vector<double> AI;
        while (std::getline(SSS, TMP, ' '))
        {
            AI.push_back(std::stod(TMP));
        }
        VoxelLabel.push_back(static_cast<std::uint32_t>(AI[AI.size()-1]));
    }
    AnthoerReader.close();

	std::uint32_t VoxelNum = 0;
    std::uint32_t VoxelPointCloudSize = 0;
	std::uint32_t PointNum2ClassifyRightly = 0;
	for (auto& SV : vSupervoxels)
	{
		SV.second->centroid_.a = VoxelLabel[VoxelNum++];
		for (auto& Point : SV.second->voxels_->points)
		{
			if (Point.a == SV.second->centroid_.a)PointNum2ClassifyRightly++;
		}
		VoxelPointCloudSize += SV.second->voxels_->points.size();
	}

	std::cout << "classifyResultAcc:" << static_cast<float>(PointNum2ClassifyRightly) / VoxelPointCloudSize << std::endl;
    std::string Timestamp = std::to_string(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
	writeSupervoxelWithCentroidLabel2TXT(vSupervoxels, Timestamp+"visualClassifyResult.txt");
}

SAABB CAABBEstimation::compute() const
{
    SAABB Box;
    for (std::uint32_t i = 0; i < m_pCloud->points.size(); i++)
    {
        if (m_pCloud->points[i].x < Box._Min[0])Box._Min[0] = m_pCloud->points[i].x;
        if (m_pCloud->points[i].x > Box._Max[0])Box._Max[0] = m_pCloud->points[i].x;
        if (m_pCloud->points[i].y < Box._Min[1])Box._Min[1] = m_pCloud->points[i].y;
        if (m_pCloud->points[i].y > Box._Max[1])Box._Max[1] = m_pCloud->points[i].y;
        if (m_pCloud->points[i].z < Box._Min[2])Box._Min[2] = m_pCloud->points[i].z;
        if (m_pCloud->points[i].z > Box._Max[2])Box._Max[2] = m_pCloud->points[i].z;
    }
    return Box;
}

void writeSingleFeature2TXT(const std::string& vOutputFileName, PointCloudT::Ptr viPointCloud, std::unordered_map<int, float>& viDONFeature)
{
    std::ofstream Writer(vOutputFileName);
    for (int i = 0; i < viPointCloud->points.size(); i++)
    {
        PointT Point = viPointCloud->points.at(i);

        Writer << static_cast<std::float_t>(Point.x) << " ";
        Writer << static_cast<std::float_t>(Point.y) << " ";
        Writer << static_cast<std::float_t>(Point.z) << " ";
        Writer << static_cast<std::uint32_t>(Point.r) << " ";
        Writer << static_cast<std::uint32_t>(Point.g) << " ";
        Writer << static_cast<std::uint32_t>(Point.b) << " ";
        Writer << static_cast<std::float_t>(viDONFeature.at(i)) << "\n";
    }
    Writer.close();
}

void mapLabelBetweenPointClouds(const PointCloudT::Ptr& vPredictedCloud, const PointCloudT::Ptr& vGroundTruthCloud)
{
    pcl::search::KdTree<PointT>::Ptr KdTree4CroundTruthCloud;
    auto __initializeKdTree = [&]()
    {
        _HIVE_EARLY_RETURN(vGroundTruthCloud && vGroundTruthCloud->empty(), "Point cloud need to be initialized with non-empty cloud!", false);
        if (KdTree4CroundTruthCloud && KdTree4CroundTruthCloud->getInputCloud() == vGroundTruthCloud)return true;
        KdTree4CroundTruthCloud = std::make_shared<pcl::search::KdTree<PointT>>();
        KdTree4CroundTruthCloud->setInputCloud(vGroundTruthCloud);
        return true;
    };

    auto __searchNeighbours = [&](const PointT& vPoint, std::vector<int>& voNeighbourPointIdxs)
    {
        _HIVE_EARLY_RETURN(KdTree4CroundTruthCloud && KdTree4CroundTruthCloud->getInputCloud()->empty(), "KDTree isn't be initialized!", 0);
        voNeighbourPointIdxs.clear();
        std::vector<float> TempVector;
        int NeighbourNum = 1;
        KdTree4CroundTruthCloud->nearestKSearch(vPoint, NeighbourNum, voNeighbourPointIdxs, TempVector);
        return voNeighbourPointIdxs.at(0);
    };

    __initializeKdTree();

    std::vector<int> GroundTruthLabels;
    std::vector<int> NeighbourPointIdxs;
    for (auto& Point : vPredictedCloud->points)
    {
        GroundTruthLabels.push_back((int)(vGroundTruthCloud->points.at(__searchNeighbours(Point, NeighbourPointIdxs))).a);
    }

    std::ofstream Writer("True&PredictedLabels.txt");
    for (int i = 0; i < vPredictedCloud->points.size(); i++)
    {
        PointT Point = vPredictedCloud->points.at(i);
        Writer << Point.x << " ";
        Writer << Point.y << " ";
        Writer << Point.z << " ";
        Writer << (int)(Point.r) << " ";
        Writer << (int)(Point.g) << " ";
        Writer << (int)(Point.b) << " ";
        Writer << (int)(Point.a) << " ";
        Writer << GroundTruthLabels[i] << " ";
        Writer << "\n";
    }
    Writer.close();
}

void writeSingleFeature2TXT(const std::string& vOutputFileName, PointCloudT::Ptr viPointCloud, std::unordered_map<int, std::vector<float>>& viCovFeature)
{
    std::ofstream Writer(vOutputFileName);
    for (int i = 0; i < viPointCloud->points.size(); i++)
    {
        PointT Point = viPointCloud->points.at(i);
        Writer << static_cast<std::float_t>(Point.x) << " ";
        Writer << static_cast<std::float_t>(Point.y) << " ";
        Writer << static_cast<std::float_t>(Point.z) << " ";
        Writer << static_cast<std::uint32_t>(Point.r) << " ";
        Writer << static_cast<std::uint32_t>(Point.g) << " ";
        Writer << static_cast<std::uint32_t>(Point.b) << " ";
        for (int k = 0; k < viCovFeature.at(i).size(); k++)
        {
            Writer << static_cast<std::float_t>(viCovFeature.at(i)[k]) << " ";
        };
        Writer << "\n";
    }
    Writer.close();
}
