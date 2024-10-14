#include "pch.h"
#include "Common.h"
#include "ModelReuseSelector.h"

int main()
{
	CModelReuseSelector PointCloudSelector;
	std::string ORFModelPath = "E:\\PointCloudDataSet\\sum\\model\\model";
	std::string NewZonePath = "E:\\PointCloudDataSet\\sum\\Scene";
	int MatureORFNum = 5;
	int NewZoneNum = 20;

	PointCloudSelector.selectSuitablestORFModel(ORFModelPath, NewZonePath, MatureORFNum, NewZoneNum);

	//PointCloudSelector.selectSuitablestORFModelWithSemanticPercent(ORFModelPath, NewZonePath);
	return 0;
}