#include "pch.h"
#include "ComputeAcc.h"

void CAccComputer::computeAcc(std::string& viResult, std::string& viGT)
{
	std::vector<int> Result, GT;
	__loadPointCloud(viResult, Result);
	__loadPointCloud(viGT, GT);
	float Correct = 0;
	for (int i = 0; i < GT.size(); i++)
		if (GT[i] == Result[i])Correct++;
	std::cout << std::to_string(Correct / GT.size()) << " ";
}



void CAccComputer::__loadPointCloud(std::string& vFilePath, std::vector<int>& voLables)
{
	std::ifstream Reader(vFilePath);
	std::string Line;
	int Index = 0;
	while (std::getline(Reader, Line))
	{
		std::stringstream SS(Line);
		std::string TMP;
		std::vector<double> String2Double;
		while (std::getline(SS, TMP, ' '))
		{
			String2Double.push_back(std::stod(TMP));
		}
		voLables.push_back(static_cast<std::uint32_t>(String2Double[6]));
		Index++;
	}
	Reader.close();
}