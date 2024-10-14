#pragma once
#include "Common.h"

class CAccComputer
{
public:
	CAccComputer() = default;
	virtual ~CAccComputer() = default;

	void computeAcc(std::string& viResult, std::string& viGT);
private:
	
	void __loadPointCloud(std::string& vFilePath,std::vector<int>& voLables);
};