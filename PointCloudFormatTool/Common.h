#pragma once

template <typename FeatureType>
void loadFeature(std::string& vFilePath, std::vector<FeatureType>& voFeature, int viIndex)
{
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
        voFeature.push_back(String2Double[viIndex]);
    }
    Reader.close();
};

void loadZoneSemanticRate(std::string& vFilePath, std::vector<float>& voZoneSemanticRate);

//void loadRGBFeature(std::string& vFilePath, std::vector<int>& voFeature)
//{
//    std::ifstream Reader(vFilePath);
//    std::string Line;
//    while (std::getline(Reader, Line))
//    {
//        std::stringstream SS(Line);
//        std::string TMP;
//        std::vector<double> String2Double;
//        while (std::getline(SS, TMP, ' '))
//        {
//            String2Double.push_back(std::stod(TMP));
//        }
//        int Value = (String2Double[0] / 16) * 16 * 15 + (String2Double[1] / 16) * 15 + (String2Double[2] / 16);
//        voFeature.push_back(Value);
//    }
//    Reader.close();
//};

