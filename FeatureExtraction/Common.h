#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZRGBL PCLPointType;
typedef pcl::PointCloud<PCLPointType> PCLPointCloudType;
typedef pcl::PointXYZRGB DoNInputPointType;
typedef pcl::PointCloud<DoNInputPointType> DoNInputPointCloudType;
typedef pcl::PointXYZ NormalEstimateInputPointType;
typedef pcl::PointCloud<NormalEstimateInputPointType> NormalEstimateInputPointCloudType;
typedef pcl::Normal NormalType;
typedef pcl::PointCloud<NormalType> NormalEstimateOutputPointCloudType;
typedef pcl::PointXYZRGBNormal PointNE;
typedef pcl::PointCloud<PointNE> PointCloudNE;
typedef pcl::PointXYZ PointCoord;
typedef pcl::PointCloud<PointCoord> PointCloudCoord;
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointNCloudT;
typedef pcl::PointXYZL PointLT;
typedef pcl::PointCloud<PointLT> PointLCloudT;

struct  SVehicleConfig
{

    int _NumFeatures = 4;
    int _FirstFeatureColumnIndex = 6;
    int _LabelIndex = 10;
    int _ConfidenceColumnIndex = 11;
};
struct  SGroundConfig
{

    int _NumFeatures = 3;
    int _FirstFeatureColumnIndex = 6;
    int _LabelIndex = 9;
    int _ConfidenceColumnIndex = 10;
};
struct  STreeConfig
{

    int _NumFeatures = 3;
    int _FirstFeatureColumnIndex = 6;
    int _LabelIndex = 9;
    int _ConfidenceColumnIndex = 10;
};
struct  SBuildingConfig
{

    int _NumFeatures = 2;
    int _FirstFeatureColumnIndex = 6;
    int _LabelIndex = 8;
    int _ConfidenceColumnIndex = 9;
};

struct SAABB
{
    SAABB() = default;
	Eigen::Vector3f _Min = { FLT_MAX,FLT_MAX ,FLT_MAX };
	Eigen::Vector3f _Max = { -FLT_MAX,-FLT_MAX,-FLT_MAX };

    inline bool isValid() const { return _Min[0] != FLT_MAX && _Min[1] != FLT_MAX && _Min[2] != FLT_MAX && _Max[0] != -FLT_MAX && _Max[1] != -FLT_MAX && _Max[2] != -FLT_MAX; }
};

struct SFeaturesBasedEigenvalue
{
    double _Curvature = 0;
    double _Linearity = 0;
    double _Planarity = 0;
    double _Scattering = 0;
    double _Anisotropy = 0;

    std::vector<double> getFeatures() const
    {
        std::vector<double> Features;
        Features.emplace_back(_Curvature);
        Features.emplace_back(_Linearity);
        Features.emplace_back(_Planarity);
        Features.emplace_back(_Scattering);
        Features.emplace_back(_Anisotropy);
        return Features;
    }

    void comFeatures(const std::array<double, 3>& vEigenvalues)
    {
        double LambdaOne = vEigenvalues[0];
        double LambdaTwo = vEigenvalues[1];
        double LambdaThree = vEigenvalues[2];

        _Curvature = (LambdaOne + LambdaTwo + LambdaThree) != 0 ? (LambdaThree / (LambdaOne + LambdaTwo + LambdaThree)) : 0;;
        _Linearity = LambdaOne != 0 ? ((LambdaOne - LambdaTwo) / LambdaOne) : 0;
        _Planarity = LambdaOne != 0 ? ((LambdaTwo - LambdaThree) / LambdaOne) : 0;
        _Scattering = LambdaThree != 0 ? (LambdaOne / LambdaThree) : 0;
        _Anisotropy = LambdaThree != 0 ? ((LambdaThree - LambdaOne) / LambdaThree) : 0;
    }
};

struct SRGBColor
{
    float R = 0.0f;
    float G = 0.0f;
    float B = 0.0f;
};

struct SFeatures
{
    float _ProjectElevation;
    float _ElevationDifference;
    float _ProjectElevationDifference;
    float _SizeOfElevationDifferenceCluster;
    float _ColorEntroy;
    float _DoN;
    std::array<float, 3> _LAB;
	float _Planarity;
	float _ZDensity;

    SFeatures(float vProjectElevation, float vElevationDifference, float vProjectElevationDifference, float vSizeOfElevationDifferenceCluster,float vColorEntroy, float vDoN, const std::array<float, 3>& vLAB, float vPlanarity, float vZDensity) :
        _ProjectElevation(vProjectElevation),
        _ElevationDifference(vElevationDifference),
        _ProjectElevationDifference(vProjectElevationDifference),
        _SizeOfElevationDifferenceCluster(vSizeOfElevationDifferenceCluster),
        _ColorEntroy(vColorEntroy),
        _DoN(vDoN),
        _LAB(vLAB),
        _Planarity(vPlanarity),
        _ZDensity(vZDensity) {}
};

struct SSupervoxelVFeatures
{
    std::uint32_t _Label;
    SFeaturesBasedEigenvalue _FeaturesBasedEigenvalue;
    double _LocalDensity;
    Eigen::VectorXf _PFH;
    double _Direction;
    std::array<float,3> _LAB;
    double _RelativeElevation;
    std::uint32_t _SemanticCategory;
};

struct SCell {
    std::uint32_t _Id = 0;
    std::vector<PointT> _cloud;
};

class CAABBEstimation
{
public:
    CAABBEstimation(PointCloudT::Ptr vCloud) :m_pCloud(vCloud) {};
    SAABB compute() const;
private:
    PointCloudT::Ptr m_pCloud;
};

void comMinMax4XYZ(const PointCloudT::Ptr& vCloud, std::pair<pcl::PointXYZ, pcl::PointXYZ>& vMinMaxCoordinate);
void computeAccByMapPredictedCloud2Groundtruth(const PointCloudT::Ptr& vPredictedCloud,const PointCloudT::Ptr& vGroundTruthCloud);
void genSupervoxelCloud(const std::map <std::uint32_t, pcl::Supervoxel<PointT>::Ptr >& vSupervoxels, PointCloudT& voSupervoxelCloud);
void loadCloud(const std::string& vCloudTXTFilePath, PointCloudT::Ptr& voCloud);
void loadClassifyResultAndComputeAcc(const std::string& vClassifyResultTXTPath,std::map<std::uint32_t, std::shared_ptr<pcl::Supervoxel<PointT>>> vSupervoxels);
void writeFeature2TXT(const std::map<std::uint32_t, SSupervoxelVFeatures>& vSVFeatureList, const std::string& vOutputFileName);
void writeFeature2LIBSVM(const std::map<std::uint32_t, SSupervoxelVFeatures>& vSVFeatureList, const int vSemanticCategoryNum, const std::string& vOutputFileName);
void writeFeatureWithPoint2TXT(const std::map <std::uint32_t, pcl::Supervoxel<PointT>::Ptr >& vSupervoxels,const std::map<std::uint32_t, SSupervoxelVFeatures>& vSVFeatureList, const std::string& vOutputFileName);
void writeSupervoxelWithPointLabel2TXT(const std::map<std::uint32_t, std::shared_ptr<pcl::Supervoxel<PointT>>>& vSupervoxels, const std::string& vOutputFileName);
void writeSupervoxelWithCentroidLabel2TXT(const std::map<std::uint32_t, std::shared_ptr<pcl::Supervoxel<PointT>>>& vSupervoxels, const std::string& vOutputFileName);
void writeSingleFeature2TXT(const std::string& vOutputFileName, PointCloudT::Ptr viPointCloud, std::unordered_map<int, std::vector<float>>& viCovFeature);
void writeSingleFeature2TXT(const std::string& vOutputFileName, PointCloudT::Ptr viPointCloud, std::unordered_map<int, float>& viDONFeature);
void mapLabelBetweenPointClouds(const PointCloudT::Ptr& vPredictedCloud, const PointCloudT::Ptr& vGroundTruthCloud);


