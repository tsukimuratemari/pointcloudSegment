#include "pch.h"
#include "Common.h"
#include "ModelReuseSelector.h"

void CModelReuseSelector::selectSuitablestORFModel(std::string& vORFModelPath, std::string& vNewZonePath, int viMatureORFNum, int viNewZoneNum)
{
	int IntervalScale = 100;
	int FistFeatureIndex = 3;
	int FeatureNum = 12;
	std::vector<int> FeatureIndex{ 0,1,2,3,4,5,6,7,8,9,10,11,12 };

	for (int m = 0; m < viNewZoneNum; m++)
	{
		std::vector<float> ZoneBhattacharyyaCoefficient(viMatureORFNum, 0.0f);
		std::cout << "New Zone" << std::to_string(m) << ":" << std::endl;
		for (int k = 0; k < FeatureIndex.size(); k++)
		{
			std::cout << "Feature Index " << std::to_string(FeatureIndex[k]) << ":  ";
			std::vector<std::vector<float>> AllFeature;
			for (int i = 1; i <= viMatureORFNum; i++)
			{
				std::vector<float> Feature;
				std::string Path = vORFModelPath + std::to_string(i) + "\\AllFeatures.txt";
				loadFeature(Path, Feature, FistFeatureIndex + FeatureIndex[k]);
				AllFeature.push_back(Feature);
			}
			CPointCloudHistogram<float> PointCloudHistogram(IntervalScale, AllFeature);
			std::vector<float> NewZoneFeature;
			std::string NewZonePath = vNewZonePath + std::to_string(m) + "\\AllFeatures.txt";
			std::vector<float> SingleFeature;
			loadFeature(NewZonePath, NewZoneFeature, FistFeatureIndex + FeatureIndex[k]);
			SingleFeature = PointCloudHistogram.computeHistogram(NewZoneFeature);
			bool IsModelPredictSameScene = false;
			for (int i = 0; i < viMatureORFNum; i++)
			{
				ZoneBhattacharyyaCoefficient[i] += SingleFeature[i];
				if (SingleFeature[i] == 1.0f)IsModelPredictSameScene = true;
				std::cout << std::setw(4) << SingleFeature[i] << " ";
			}
			if (IsModelPredictSameScene)
			{
				std::cout << std::endl;
				break;
			}
			std::cout << std::endl;
		}

		float Max = 0.0;
		int SameZoneIndex = 0;
		std::cout << "Average: ";
		for (int i = 0; i < viMatureORFNum; i++)
		{
			ZoneBhattacharyyaCoefficient[i] /= FeatureIndex.size();
			if (Max < ZoneBhattacharyyaCoefficient[i])
			{
				Max = ZoneBhattacharyyaCoefficient[i];
				SameZoneIndex = i;
			}
			std::cout << ZoneBhattacharyyaCoefficient[i] << " ";
		}
		std::cout << std::endl;
		std::cout << SameZoneIndex + 1 << std::endl;
	}
};

void CModelReuseSelector::selectSuitablestORFModelWithSemanticPercent(std::string& vORFModelPath, std::string& vNewZonePath, int viMatureORFNum, int viNewZoneNum)
{
	//第二次只需要构建一个直方图，直方图有4个维度 分别为道路、树木、建筑、汽车语义的占比
	int IntervalScale = 4;

	int LabelIndex = 6;
	std::vector<std::vector<float>> MatureORFZoneHistogram;
	for (int i = 0; i < viMatureORFNum; i++)
	{
		std::vector<float> ZoneSemanticRate(4, 1e-6);
		std::string Path = vORFModelPath + std::to_string(i) + "\\PostPro.txt";
		loadZoneSemanticRate(Path, ZoneSemanticRate);
		MatureORFZoneHistogram.push_back(ZoneSemanticRate);
	}
	
	

	for (int m = 1; m <= viNewZoneNum; m++)
	{
		std::vector<float> BhattacharyyaCoefficientVector;

		std::vector<float> NewZoneSemanticRate(4, 1e-6);
		std::string Path = vNewZonePath + std::to_string(m) + "\\PostPro.txt";
		loadZoneSemanticRate(Path, NewZoneSemanticRate);
		

		for (int i = 0; i < viMatureORFNum; i++)
		{
			float BhattacharyyaCoefficient = 0.0f;
			for (int k = 0; k < IntervalScale; k++)
			{
				BhattacharyyaCoefficient += sqrt(NewZoneSemanticRate[k] * MatureORFZoneHistogram[i][k]);
			}
			BhattacharyyaCoefficientVector.push_back(BhattacharyyaCoefficient);
		}
		
		std::cout << "New Zone " << std::to_string(m) << " :" << std::endl;
		for (int i = 0; i < viMatureORFNum; i++)
		{
			std::cout << std::setw(4) << BhattacharyyaCoefficientVector[i] << " ";
		}
		std::cout << std::endl;
	}
}