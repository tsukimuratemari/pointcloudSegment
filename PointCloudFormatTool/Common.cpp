#include "pch.h"
#include "Common.h"

void loadZoneSemanticRate(std::string& vFilePath, std::vector<float>& voZoneSemanticRate)
{
    int LabelIndex = 6, GroundIndex = 0, TreeIndex = 1, BuildingIndex = 2, VehicleIndex = 3;
    std::ifstream Reader(vFilePath);
    std::string Line;
    while (std::getline(Reader, Line))
    {
        std::stringstream SS(Line);
        std::string TMP;
        std::vector<double> String2Double;
        while (std::getline(SS, TMP, ' '))
        {
            String2Double.push_back(std::stod(TMP));
        }
        switch (int(String2Double[LabelIndex]))
        {
        case 0:
            voZoneSemanticRate[GroundIndex]++;
            break;
        case 2:
            voZoneSemanticRate[TreeIndex]++;
            break;
        case 3:
            voZoneSemanticRate[BuildingIndex]++;
            break;
        case 4:
            voZoneSemanticRate[VehicleIndex]++;
            break;
        default:
            break;
        }
    }
    Reader.close();

    int PointsNum = voZoneSemanticRate[GroundIndex] + voZoneSemanticRate[TreeIndex] +
        voZoneSemanticRate[BuildingIndex] + voZoneSemanticRate[VehicleIndex];

    voZoneSemanticRate[GroundIndex] /= PointsNum;
    voZoneSemanticRate[TreeIndex] /= PointsNum;
    voZoneSemanticRate[BuildingIndex] /= PointsNum;
    voZoneSemanticRate[VehicleIndex] /= PointsNum;

};