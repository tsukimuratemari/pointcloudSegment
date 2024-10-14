#pragma once
#include <iostream>
#include <sstream>
#include <assert.h>
#include <iomanip>
#include <vector>
#include "boost/filesystem.hpp"
#include "data.h"
#include "classifier.h"

using namespace std;

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

enum class EBiClassificationSemanticCategory :uint32_t
{
	NegativeCategory = 0,
	PositiveCategory = 1
};

class CPointCloudTool {
public:
	CPointCloudTool() = default;
	~CPointCloudTool() = default;

	void changeLabelAndConf2AnotherDataset(std::vector<std::vector<double>>& vDataset, const std::vector<std::vector<double>>& vLastResult, int vLabelIndex4AnotherDataset = -1);
	void binarizeDLTrainData(const ESemanticCategory& vGenerateSemanticCategory, vector<vector<double>>& vioTrainData);
	void simplifyDLResult(const string& vDLResultPath, const ESemanticCategory& vGenerateSemanticCategory, vector<vector<double>>& voTrainData);
	void generateORFDataSet(vector<vector<double>>& vioTrainData, vector<vector<double>>& voTestData, const uint32_t vFirstColumnIndexOfFeatures, const uint32_t vLastColumnIndexOfFeatures);
	void visualizeORFApproachDLResult(vector<vector<double>>& vioSimplifiedTrainData, const vector<Result>& vORFResult);

	void mergeORFManualResult(const vector<string>& vPosManualResultPathVector, const vector<string>& vNegManualResultPathVector, vector<vector<double>>& voTrainData);
	void mergeMultiORFManualResult(const vector<string>& vPosManualResultPathVector, const vector<string>& vNegManualResultPathVector, vector<vector<double>>& voTrainData);
	void visualizeORFApproachManualResult(vector<vector<double>>& vioManualTrainData, const ESemanticCategory& vGenerateSemanticCategory, const vector<Result>& vORFResult);
	void visualizeMultiORFApproachManualResult(vector<vector<double>>& vioManualTrainData,const vector<Result>& vORFResult);

	
	void setLabelColumnIndex(int vLabelColumnIndex) { m_LabelColumnIndex = vLabelColumnIndex; }
	void setConfidenceColumnIndex(int vConfidenceColumnIndex) { m_ConfidenceColumnIndex = vConfidenceColumnIndex; }

	void mapPreditcedLabel2OriginData(const string& vOriginDataPath, vector<vector<double>>& vioPredictedResult);
	void loadOriginArray(const string& vInputFilePath, vector<vector<double>>& voArray);
	void deleteDuplicateConfidence(vector<vector<double>>& voArray);
	void loadArrayFile(const string& vInputFilePath, vector<vector<double>>& voArray);
	void saveArrayFile(const string& vOutputFilePath, const vector<vector<double>>& viArray);
	void generateFinalResult(vector<vector<double>>& vioArray);
	void approachGTResult(const vector<Result>& vGround, const vector<Result>& vTree, const vector<Result>& vBuilding, const vector<Result>& vVehicle, vector<uint32_t>& vApproachResult);

private:
	uint32_t m_FirstFeatureColumnIndex = 6;
	uint32_t m_LastFeatureColumnIndex = 13;
	uint32_t m_SVSerialColumnIndex = 11;
	uint32_t m_LabelColumnIndex = 14;
	uint32_t m_ConfidenceColumnIndex = 15;

	void __sliceArray(const vector<vector<double>>& viOriginArray, vector<vector<double>>& voSliceArray, vector<uint32_t>& viIndex, const bool vIsRow);
	void __uniqueArray(vector<vector<double>>& vioArray, vector<uint32_t>voIndices, uint32_t vUniqueColumnIndex);
	void __sortArray(vector<vector<double>>& vioArray, uint32_t vSortColumnIndex);
	void __binarizeDLTrainData(vector<vector<double>>& vioTrainData, const uint32_t vPositiveSemanticColumnIndex, const ESemanticCategory& vPositiveSemanticCategory);
	void __binarizeManualTrainData(vector<vector<double>>& vioTrainData, const uint32_t vPositiveSemanticColumnIndex, const bool vIsPositive);
	void __stackArray(const vector<vector<double>>& viArray1, const vector<vector<double>>& viArray2, vector<vector<double>>& voResultArray, const bool vIsRow);
};


