#pragma once
#include "Common.h"
#include "Utils.h"
#include "experimenter.h"
#include "online_rf.h"
#include "ProjectionElevation.h"
#include "ElevationDifference.h"
#include "ColorEntropy.h"
#include "DoN.h"
#include "LAB.h"
#include "Covariance.h"
#include "ZDirectionDensity.h"

static SVehicleConfig VehicleConfig;
static SGroundConfig GroundConfig;
static SBuildingConfig BuildingConfig;
static STreeConfig TreeConfig;
static 	std::vector<std::vector<ESemanticCategory>> SCENESEMINFOLIST =
{
	{ESemanticCategory::Vehicle,ESemanticCategory::Ground,ESemanticCategory::Tree,ESemanticCategory::Building },
	{ESemanticCategory::Vehicle,ESemanticCategory::Ground,ESemanticCategory::Tree,ESemanticCategory::Building },
	{ESemanticCategory::Vehicle,ESemanticCategory::Ground,ESemanticCategory::Tree,ESemanticCategory::Building },
	{ESemanticCategory::Vehicle,ESemanticCategory::Ground,ESemanticCategory::Tree,ESemanticCategory::Building },
	{ESemanticCategory::Vehicle,ESemanticCategory::Ground,ESemanticCategory::Tree,ESemanticCategory::Building },
	{ESemanticCategory::Vehicle,ESemanticCategory::Ground,ESemanticCategory::Tree,ESemanticCategory::Building },
	{ESemanticCategory::Vehicle,ESemanticCategory::Ground,ESemanticCategory::Building },
	{ESemanticCategory::Vehicle,ESemanticCategory::Ground,ESemanticCategory::Tree,ESemanticCategory::Building },
	{ESemanticCategory::Vehicle,ESemanticCategory::Ground,ESemanticCategory::Tree,ESemanticCategory::Building },
	{ESemanticCategory::Vehicle,ESemanticCategory::Ground,ESemanticCategory::Building },
	{ESemanticCategory::Vehicle,ESemanticCategory::Ground,ESemanticCategory::Tree,ESemanticCategory::Building },
	{ESemanticCategory::Vehicle,ESemanticCategory::Ground,ESemanticCategory::Tree,ESemanticCategory::Building },
	{ESemanticCategory::Vehicle,ESemanticCategory::Ground,ESemanticCategory::Building },
	{ESemanticCategory::Vehicle,ESemanticCategory::Ground,ESemanticCategory::Building },
	{ESemanticCategory::Vehicle,ESemanticCategory::Ground,ESemanticCategory::Tree,ESemanticCategory::Building },
	{ESemanticCategory::Vehicle,ESemanticCategory::Ground,ESemanticCategory::Tree,ESemanticCategory::Building },
	{ESemanticCategory::Vehicle,ESemanticCategory::Ground,ESemanticCategory::Building },
	{ESemanticCategory::Building,ESemanticCategory::Ground },
	{ESemanticCategory::Vehicle,ESemanticCategory::Ground,ESemanticCategory::Tree,ESemanticCategory::Building },
	{ESemanticCategory::Vehicle,ESemanticCategory::Tree,ESemanticCategory::Building },
	{ESemanticCategory::Vehicle,ESemanticCategory::Ground,ESemanticCategory::Tree,ESemanticCategory::Building },
	{ESemanticCategory::Building,ESemanticCategory::Ground},
	{ESemanticCategory::Vehicle,ESemanticCategory::Ground,ESemanticCategory::Tree,ESemanticCategory::Building },
	{ESemanticCategory::Vehicle,ESemanticCategory::Ground,ESemanticCategory::Tree,ESemanticCategory::Building },
	{ESemanticCategory::Vehicle,ESemanticCategory::Ground,ESemanticCategory::Tree,ESemanticCategory::Building },
	{ESemanticCategory::Vehicle,ESemanticCategory::Ground,ESemanticCategory::Tree,ESemanticCategory::Building },
	{ESemanticCategory::Vehicle,ESemanticCategory::Ground,ESemanticCategory::Tree,ESemanticCategory::Building },
	{ESemanticCategory::Building, ESemanticCategory::Ground },
	{ESemanticCategory::Building, ESemanticCategory::Ground },
	{ESemanticCategory::Tree,ESemanticCategory::Ground,ESemanticCategory::Building },
};

class CSmallScalePostProcessing
{
public:
	CSmallScalePostProcessing(std::string vFilePath);
	~CSmallScalePostProcessing();
	void loadPointCloud(std::string& vFilePath);
	void savePointCloud(std::string& vFilePath);
	void savePointCloud(std::string& vFilePath, std::vector<int> viPointCloudIndexVector);
	void clusteringPointCloud(std::vector<std::vector<int>>& voClusterResult);
	
	void mixDLAndManualConfidenceResult(std::vector<std::vector<double>>& viClusterResult);
	void computeAcc(PCLPointCloudType::Ptr& viPredicted, std::string& viGTPath);
	PCLPointCloudType::Ptr pPointCloud = nullptr;
	std::map<int, float> m_Confidence;
private:
	void __preUnclassifiedPoints();

};


class CPCTool
{
public:
	std::string ConfFilePath = "..\\..\\OnlineLearning\\conf\\omcb.conf";

	std::string TESTDATADIR;
	std::string InitialPath;
	std::string FeaturePath;
	std::string VehicleFeaturePath;
	std::string GroundFeaturePath;
	std::string BuildingFeaturePath;
	std::string TreeFeaturePath;

	std::string SubsequentDir;
	std::string  ManualResultPath;

	CPointCloudTool PointCloudTool;

	CPCTool() { m_pCloud = std::make_shared<PointCloudT>(); }

	~CPCTool() = default;
	
	void extractFeatures();

	void trainORFModel(std::vector<std::pair<std::string, std::vector<ESemanticCategory>>>& viTrainDir, std::vector<std::vector<std::string>>& viTestDir);

	void setInitialPath(std::string& viTESTDATADIR);

	void setSubsequentDir(std::string& viSubsequentDir);

	std::string returnSemanticString(ESemanticCategory viSemantic);

private:

	PointCloudT::Ptr m_pCloud;

	void __writeTreeFeatures(const std::unordered_map<int, SFeatures>& vFeatures, const std::string& vPath);

	void __writeBuildingFeatures(const std::unordered_map<int, SFeatures>& vFeatures, const std::string& vPath);

	void __writeVehicleFeatures(const std::unordered_map<int, SFeatures>& vFeatures, const std::string& vPath);

	void __writeGroundFeatures(const std::unordered_map<int, SFeatures>& vFeatures, const std::string& vPath);

	void __writePositionAndFeatures(const std::unordered_map<int, SFeatures>& vFeatures, const std::string& vPath);

	
};