#pragma once
#include "PointCloud.h"

class CModelReuseSelector
{
public:
	CModelReuseSelector() = default;

	void selectSuitablestORFModel(std::string& vORFModelPath, std::string& vNewZonePath, int viMatureORFNum, int viNewZoneNum);

	void selectSuitablestORFModelWithSemanticPercent(std::string& vORFModelPath, std::string& vNewZonePath, int viMatureORFNum, int viNewZoneNum);

private:

};
