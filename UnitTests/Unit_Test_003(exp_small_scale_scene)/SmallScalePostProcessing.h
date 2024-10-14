#pragma once
#include "pch.h"
#include "Common.h"

enum class ESemanticCategory :uint32_t
{
	Ground = 0,
	Vegetation = 1,
	Tree = 2,
	Building = 3,
	Vehicle = 4,
	River = 5,
	Undefined = 6
};

class CSmallScalePostProcessing 
{
public: 
	CSmallScalePostProcessing(std::string vFilePath, int vLabelIndex);
	~CSmallScalePostProcessing();
	void loadPointCloud(std::string& vFilePath, int vLabelIndex);
	void savePointCloud(std::string& vFilePath);
	void savePointCloud(std::string& vFilePath, std::vector<int> viPointCloudIndexVector);
	void clusteringPointCloud(std::vector<std::vector<int>>& voClusterResult);
	void processSmallScaleCluster(std::vector<std::vector<int>>& viClusterResult);
	void mixDLAndManualConfidenceResult(std::vector<std::vector<double>>& viClusterResult);
	void computeAcc(PCLPointCloudType::Ptr& viPredicted, std::string& viGTPath);
	PCLPointCloudType::Ptr pPointCloud=nullptr;
	std::map<int, float> m_Confidence;
private:
	void __preUnclassifiedPoints();

};