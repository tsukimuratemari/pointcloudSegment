#include "Utils.h"
#include <filesystem>


//void CPointCloudTool::simplifyDLResult(const string& vDLResultPath, const ESemanticCategory& vGenerateSemanticCategory, vector<vector<double>>& voTrainData)
//{
//    voTrainData.clear();
//    loadArrayFile(vDLResultPath, voTrainData);
//    vector<uint32_t> UniqueIndexVector;
//    __uniqueArray(voTrainData, UniqueIndexVector, m_SVSerialColumnIndex);
//    __binarizeDLTrainData(voTrainData, m_LabelColumnIndex, vGenerateSemanticCategory);
//}

void CPointCloudTool::changeLabelAndConf2AnotherDataset(std::vector<std::vector<double>>& vDataset, const std::vector<std::vector<double>>& vAnotherDataset, int vLabelIndex4AnotherDataset)
{
    if (vDataset.size() != vAnotherDataset.size())std::cerr << "The size don't matching!" << std::endl;
    if (vDataset.empty() && vDataset[0].empty())std::cerr << "Empty Dataset!" << std::endl;
    if (vLabelIndex4AnotherDataset == -1)vLabelIndex4AnotherDataset = vAnotherDataset[0].size() - 3;

    for (int i = 0; i < vDataset.size(); i++)
    {
        vDataset[i][m_LabelColumnIndex] = vAnotherDataset[i][vLabelIndex4AnotherDataset];
        double Confidence = vAnotherDataset[i][vLabelIndex4AnotherDataset + 1];
        if (Confidence >= 1.0f)
            Confidence = 0.999999;
        vDataset[i][m_LabelColumnIndex + 1] = Confidence;
    }

}

void CPointCloudTool::binarizeDLTrainData(const ESemanticCategory& vGenerateSemanticCategory, vector<vector<double>>& vioTrainData)
{
    for (int i = 0; i < vioTrainData.size(); i++)
    {
        if (static_cast<uint32_t>(vioTrainData[i][m_LabelColumnIndex]) == static_cast<uint32_t>(vGenerateSemanticCategory))
        {
            vioTrainData[i][m_LabelColumnIndex] = static_cast<double>(EBiClassificationSemanticCategory::PositiveCategory);
        }
        else
        {
            vioTrainData[i][m_LabelColumnIndex] = static_cast<double>(EBiClassificationSemanticCategory::NegativeCategory);
        }
    }
}

void CPointCloudTool::loadOriginArray(const string& vInputFilePath, vector<vector<double>>& voArray)
{
    voArray.clear();
    loadArrayFile(vInputFilePath, voArray);
    //vector<uint32_t> UniqueIndexVector;
    //__uniqueArray(voArray, UniqueIndexVector, m_SVSerialColumnIndex);
    for (int i = 0; i < voArray.size(); i++)
    {
        double Confidence = 1e-8;
        voArray[i].push_back(Confidence);
    }
}


//delete the duplicate confidence cloumn when call loadOriginArray() function repeatly;
void CPointCloudTool::deleteDuplicateConfidence(vector<vector<double>>& voArray)
{
    if (voArray.empty() || voArray[0].empty())
    {
        std::cerr << "The input vector is empty when call function deleteDuplicateConfidence!" << std::endl;
        return;
    }
    for (auto& Data : voArray)
    {
        Data.pop_back();
    }
}

//void CPointCloudTool::__binarizeDLTrainData(vector<vector<double>>& vioTrainData, const uint32_t vPositiveSemanticColumnIndex, const ESemanticCategory& vPositiveSemanticCategory)
//{
//    for (int i = 0; i < vioTrainData.size(); i++)
//    {
//        if (int(vioTrainData[i][vPositiveSemanticColumnIndex]) == static_cast<uint32_t>(vPositiveSemanticCategory))
//        {
//            vioTrainData[i][vPositiveSemanticColumnIndex] = static_cast<double>(EBiClassificationSemanticCategory::PositiveCategory);
//        }
//        else
//        {
//            vioTrainData[i][vPositiveSemanticColumnIndex] = static_cast<double>(EBiClassificationSemanticCategory::NegativeCategory);
//        }
//        double LabelConfidence = 1e-8;
//        vioTrainData[i].push_back(LabelConfidence);
//    }
//}

//void CPointCloudTool::generateORFDataSet(vector<vector<double>>& vioTrainData, vector<vector<double>>& voTestData, const uint32_t vFirstColumnIndexOfFeatures, const uint32_t vLastColumnIndexOfFeatures)
//{
//
//    vector<double> TestDataFirstRow;
//    TestDataFirstRow.push_back(vioTrainData.size());
//    uint32_t LabelColumn = 1;
//    TestDataFirstRow.push_back(static_cast<double>(LabelColumn));
//    voTestData.push_back(TestDataFirstRow);
//    for (int i = 0; i < vioTrainData.size(); i++)
//    {
//        vector<double> TestDataRow;
//        TestDataRow.push_back(vioTrainData[i][m_LabelColumnIndex]);
//        voTestData.push_back(TestDataRow);
//    }
//
//    vector<vector<double>> Features;
//    vector<uint32_t> SliceIndexVector;
//    for (uint32_t i = vFirstColumnIndexOfFeatures; i <= vLastColumnIndexOfFeatures; i++)SliceIndexVector.push_back(i);
//    __sliceArray(vioTrainData, Features, SliceIndexVector, false);
//
//    vector<double> TrainDataFirstRow;
//    TrainDataFirstRow.push_back(vioTrainData.size());
//    TrainDataFirstRow.push_back(vLastColumnIndexOfFeatures - vFirstColumnIndexOfFeatures + 1);
//
//    vioTrainData.clear();
//    vioTrainData.push_back(TrainDataFirstRow);
//    for (int i = 0; i < Features.size(); i++)
//        vioTrainData.push_back(Features[i]);
//}

void CPointCloudTool::visualizeORFApproachDLResult(vector<vector<double>>& vioSimplifiedTrainData, const vector<Result>& vORFResult)
{
    _ASSERT(vioSimplifiedTrainData.size() == vORFResult.size());
    for (int i = 0; i < vioSimplifiedTrainData.size(); i++)
    {
        vioSimplifiedTrainData[i][m_LabelColumnIndex] = static_cast<double>(vORFResult[i].prediction);
    }
}

void CPointCloudTool::mergeORFManualResult(const vector<string>& vPosManualResultPathVector, const vector<string>& vNegManualResultPathVector, vector<vector<double>>& voTrainData)
{
    
    voTrainData.clear();
    for (auto it : vPosManualResultPathVector)
    {
        vector<vector<double>> ManualData;
        loadArrayFile(it, ManualData);
        __binarizeManualTrainData(ManualData, m_LabelColumnIndex, true);
        voTrainData.insert(voTrainData.end(), ManualData.begin(), ManualData.end());
    }
    for (auto it : vNegManualResultPathVector)
    {
        vector<vector<double>> ManualData;
        loadArrayFile(it, ManualData);
        __binarizeManualTrainData(ManualData, m_LabelColumnIndex, false);
        voTrainData.insert(voTrainData.end(), ManualData.begin(), ManualData.end());
    }
}

void CPointCloudTool::mergeMultiORFManualResult(const vector<string>& vPosManualResultPathVector, const vector<string>& vNegManualResultPathVector, vector<vector<double>>& voTrainData)
{

    voTrainData.clear();
    for (auto it : vPosManualResultPathVector)
    {
        vector<vector<double>> ManualData;
        loadArrayFile(it, ManualData);
        voTrainData.insert(voTrainData.end(), ManualData.begin(), ManualData.end());
    }
    for (auto it : vNegManualResultPathVector)
    {
        vector<vector<double>> ManualData;
        loadArrayFile(it, ManualData);
        voTrainData.insert(voTrainData.end(), ManualData.begin(), ManualData.end());
    }
}

void CPointCloudTool::__binarizeManualTrainData(vector<vector<double>>& vioTrainData, const uint32_t vPositiveSemanticColumnIndex, const bool vIsPositive)
{
    if (vIsPositive)
    {
        for (int i = 0; i < vioTrainData.size(); i++)
        {
            vioTrainData[i][vPositiveSemanticColumnIndex] = static_cast<uint32_t>(EBiClassificationSemanticCategory::PositiveCategory);
        }
    }
    else
    {
        for (int i = 0; i < vioTrainData.size(); i++)
        {
            vioTrainData[i][vPositiveSemanticColumnIndex] = static_cast<uint32_t>(EBiClassificationSemanticCategory::NegativeCategory);
        }
    }
}

void CPointCloudTool::visualizeORFApproachManualResult(vector<vector<double>>& vioManualTrainData, const ESemanticCategory& vGenerateSemanticCategory, const vector<Result>& vORFResult)
{
    _ASSERT(vioManualTrainData.size() == vORFResult.size());
    double Confidence;
    for (int i = 0; i < vioManualTrainData.size(); i++)
    {
        Confidence = vORFResult[i].confidence.maxCoeff();
        if (Confidence >= 1.0f)
            Confidence = 0.999999;
        if (vioManualTrainData[i][m_ConfidenceColumnIndex] < Confidence)
        {
            if (static_cast<uint32_t>(vORFResult[i].prediction) == static_cast<uint32_t>(EBiClassificationSemanticCategory::PositiveCategory))
            {
                vioManualTrainData[i][m_LabelColumnIndex] = static_cast<double>(vGenerateSemanticCategory);
                vioManualTrainData[i][m_ConfidenceColumnIndex] = Confidence;
            }
            else
            {
                vioManualTrainData[i][m_LabelColumnIndex] = static_cast<double>(vioManualTrainData[i][m_LabelColumnIndex]);
                vioManualTrainData[i][m_ConfidenceColumnIndex] = static_cast<double>(vioManualTrainData[i][m_ConfidenceColumnIndex]);
            }
            
        }
    }
    
    vector<vector<double>> SaveArray;
    for (int i = 0; i < vioManualTrainData.size(); i++)
    {
        SaveArray.push_back(vioManualTrainData[i]);
    }
    vioManualTrainData = SaveArray;
}

void CPointCloudTool::visualizeMultiORFApproachManualResult(vector<vector<double>>& vioOriginResult, const vector<Result>& vORFResult)
{
    _ASSERT(vioOriginResult.size() == vORFResult.size());
    //TODO:if multi RF need compare confidence?
    for (int i = 0; i < vioOriginResult.size(); i++)
    {
        vioOriginResult[i][m_LabelColumnIndex] = static_cast<double>(vORFResult[i].prediction);
    }
}

void CPointCloudTool::mapPreditcedLabel2OriginData(const string& vOriginDataPath, vector<vector<double>>& vioPredictedResult)
{
    vector<vector<double>> DenseOriginData;
    loadArrayFile(vOriginDataPath, DenseOriginData);
    //__sortArray(DenseOriginData, m_SVSerialColumnIndex);
    //__sortArray(vioPredictedResult, m_SVSerialColumnIndex);
    int k = 0;
    for (int i = 0; i < vioPredictedResult.size(); i++)
    {
        double VoxelLabel = vioPredictedResult[i][m_LabelColumnIndex];
        uint32_t VoxelIndex = static_cast<uint32_t>(vioPredictedResult[i][m_SVSerialColumnIndex]);
        while ((k < DenseOriginData.size()) && (static_cast<uint32_t>(DenseOriginData[k][m_SVSerialColumnIndex]) == VoxelIndex))
        {
            DenseOriginData[k][m_LabelColumnIndex] = VoxelLabel;
            DenseOriginData[k].push_back(vioPredictedResult[i][m_LabelColumnIndex]);
            k++;
        }
    }
    generateFinalResult(DenseOriginData);
    vioPredictedResult = DenseOriginData;
}

void CPointCloudTool::loadArrayFile(const string& vInputFilePath, vector<vector<double>>& voArray)
{
    voArray.clear();
    ifstream Reader(vInputFilePath);
    string FileLine;
    uint32_t i = 0;
    while (getline(Reader, FileLine))
    {
        stringstream StringStream(FileLine);
        vector<double> SingleRowData;
        double Data;
        while (StringStream >> Data)
        {
            SingleRowData.push_back(Data);
        }
        voArray.push_back(SingleRowData);
        i += 1;
    }
}

void CPointCloudTool::saveArrayFile(const string& vOutputFilePath, const vector<vector<double>>& viArray)
{
    ofstream Writer(vOutputFilePath);
    Writer.setf(ios::fixed, ios::floatfield);
    Writer.precision(3);
    for (int i = 0; i < viArray.size(); i++)
    {
        for (int k = 0; k < viArray[i].size() - 1; k++)
        {
            Writer << viArray[i][k] << " ";
        }
        Writer << viArray[i][viArray[i].size() - 1];
        Writer << endl;
    }
    Writer.close();
}

void CPointCloudTool::generateFinalResult(vector<vector<double>>& vioArray)
{
    vector<vector<double>> FinalArray;
    vector<uint32_t> IndexArray;
    for (uint32_t i = 0; i < m_FirstFeatureColumnIndex; i++)IndexArray.push_back(i);
    IndexArray.push_back(m_SVSerialColumnIndex);
    IndexArray.push_back(m_LabelColumnIndex);
    __sliceArray(vioArray, FinalArray, IndexArray, false);
    vioArray = FinalArray;
}

void CPointCloudTool::approachGTResult(const vector<Result>& vGround, const vector<Result>& vTree, const vector<Result>& vBuilding, const vector<Result>& vVehicle, vector<uint32_t>& vApproachResult)
{
    if (vGround.size() != vTree.size() || vTree.size() != vBuilding.size() || vBuilding.size() != vVehicle.size())
    {
        cout << "GT Approach result are not same" << endl;
        return;
    }
    vApproachResult.clear();
    int unlabelnum = 0;
    for (int i = 0; i < vGround.size(); i++)
    {
        if (vGround[i].confidence.maxCoeff() > vTree[i].confidence.maxCoeff() && vGround[i].confidence.maxCoeff() > vBuilding[i].confidence.maxCoeff() && vGround[i].confidence.maxCoeff() > vVehicle[i].confidence.maxCoeff())
            vApproachResult.push_back(static_cast<uint32_t>(vGround[i].prediction));
        else if (vTree[i].confidence.maxCoeff() > vGround[i].confidence.maxCoeff() && vTree[i].confidence.maxCoeff() > vBuilding[i].confidence.maxCoeff() && vTree[i].confidence.maxCoeff() > vVehicle[i].confidence.maxCoeff())
            vApproachResult.push_back(static_cast<uint32_t>(vTree[i].prediction));
        else if (vBuilding[i].confidence.maxCoeff() > vGround[i].confidence.maxCoeff() && vBuilding[i].confidence.maxCoeff() > vTree[i].confidence.maxCoeff() && vBuilding[i].confidence.maxCoeff() > vVehicle[i].confidence.maxCoeff())
            vApproachResult.push_back(static_cast<uint32_t>(vBuilding[i].prediction));
        else if (vVehicle[i].confidence.maxCoeff() > vGround[i].confidence.maxCoeff() && vVehicle[i].confidence.maxCoeff() > vTree[i].confidence.maxCoeff() && vVehicle[i].confidence.maxCoeff() > vBuilding[i].confidence.maxCoeff())
            vApproachResult.push_back(static_cast<uint32_t>(vVehicle[i].prediction));
        else 
        {
            unlabelnum++;
            vApproachResult.push_back(static_cast<uint32_t>(vGround[i].prediction));
            
        }
    }
    cout << unlabelnum<<" voxels can not find exact label" << endl;
    
}

void CPointCloudTool::__uniqueArray(vector<vector<double>>& vioArray, vector<uint32_t>voIndices, uint32_t vUniqueColumnIndex)
{
    _ASSERT(vUniqueColumnIndex < vioArray.size());
    vector<vector<double>> UniqueArray;
    vector<uint32_t> UniqueColumnValueVector;
    for (int i = 0; i < vioArray.size(); i++)
    {
        uint32_t VoxelIndex = uint32_t(vioArray[i][vUniqueColumnIndex]);
        if (find(UniqueColumnValueVector.begin(), UniqueColumnValueVector.end(), VoxelIndex) == UniqueColumnValueVector.end())
        {
            UniqueArray.push_back(vioArray[i]);
            UniqueColumnValueVector.push_back(VoxelIndex);
            voIndices.push_back(i);
        }
    }
    vioArray = UniqueArray;
}

void CPointCloudTool::__sliceArray(const vector<vector<double>>& viOriginArray, vector<vector<double>>& voSliceArray, vector<uint32_t>& viIndex, const bool vIsRow)
{
    _ASSERT(viIndex.size() < viOriginArray.size());
    if (vIsRow)
    {
        for (auto it : viIndex)
        {
            voSliceArray.push_back(viOriginArray[it]);
        }
    }
    else
    {
        for (int i = 0; i < viOriginArray.size(); i++)
        {
            vector<double> SingleRow;
            for (auto it : viIndex)
            {
                SingleRow.push_back(viOriginArray[i][it]);
            }
            voSliceArray.push_back(SingleRow);
        }
    }
}

void CPointCloudTool::__sortArray(vector<vector<double>>& vioArray, uint32_t vSortColumnIndex)
{
    _ASSERT(vSortColumnIndex < vioArray.size());
    for (int i = 1; i < vioArray.size(); i++)
        for (int k = 0; k < vioArray.size() - i; k++)
        {
            if (vioArray[k][vSortColumnIndex] > vioArray[k + 1][vSortColumnIndex])
            {
                swap(vioArray[k], vioArray[k + 1]);
            }
        }
}

void CPointCloudTool::__stackArray(const vector<vector<double>>& viArray1, const vector<vector<double>>& viArray2, vector<vector<double>>& voResultArray, const bool vIsRow)
{
    if (vIsRow)
    {
        _ASSERT(viArray1[0].size() == viArray2[0].size());
        for (int i = 0; i < viArray1.size(); i++)
            voResultArray.push_back(viArray1[i]);
        for (int i = 0; i < viArray2.size(); i++)
            voResultArray.push_back(viArray2[i]);
    }
    else
    {
        _ASSERT(viArray1.size() == viArray2.size());
        for (int i = 0; i < viArray1.size(); i++)
        {
            vector<double>SingleRow;
            for (int k = 0; k < viArray1[i].size(); k++)
                SingleRow.push_back(viArray1[i][k]);
            for (int k = 0; k < viArray2[i].size(); k++)
                SingleRow.push_back(viArray2[i][k]);
            voResultArray.push_back(SingleRow);
        }
    }
}


