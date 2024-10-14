#include "pch.h"
#include "LAB.h"

void CLAB::compute(std::unordered_map<int, std::array<float, 3>>& voLABs)
{
    _ASSERTE(m_pCloud && !m_pCloud->empty());
    voLABs.reserve(m_pCloud->size());
    for (auto Itr=m_pCloud->cbegin();Itr!=m_pCloud->cend();Itr++)
    {
        std::array<float, 3> RGB = { static_cast<float>(Itr->r) ,
        static_cast<float>(Itr->g) ,
        static_cast<float>(Itr->b) };
        std::array<float, 3> LAB;
        __convertRGB2LAB(RGB, LAB);
        voLABs.emplace(std::distance(m_pCloud->cbegin(),Itr),LAB);
    }
}

void CLAB::computeBasedSV(const std::map<std::uint32_t, pcl::Supervoxel<PointT>::Ptr>& vSupervoxels, std::map<std::uint32_t, std::array<float,3>>& voLABs)
{
	for (auto& SV : vSupervoxels)
	{
        std::array<float, 3> RGB = { static_cast<float>(SV.second->centroid_.r) ,
            static_cast<float>(SV.second->centroid_.g) ,
            static_cast<float>(SV.second->centroid_.b) };
		std::array<float, 3> LAB;
        __convertRGB2LAB(RGB, LAB);
		voLABs.insert(std::make_pair(SV.first, LAB));
	}
}

void CLAB::__convertRGB2LAB(std::array<float, 3> vRGB, std::array<float, 3>& voLAB)
{
    _ASSERT(!vRGB.empty() && vRGB.size() == 3);
    vRGB[0] /= 255.0f;
    vRGB[1] /= 255.0f;
    vRGB[2] /= 255.0f;
    __correctingGamma(vRGB);

    std::array<float, 3> XYZ;
    __convertRGB2XYZ(vRGB, XYZ);
    __convertXYZ2LAB(XYZ, voLAB);
}

void CLAB::__convertRGB2XYZ(const std::array<float, 3>& vRGB, std::array<float, 3>& voXYZ)
{
    voXYZ[0] = vRGB[0] * 0.436052025 + vRGB[1] * 0.385081593 + vRGB[2] * 0.143087414;
    voXYZ[1] = vRGB[0] * 0.222491598 + vRGB[1] * 0.716886060 + vRGB[2] * 0.060621486;
    voXYZ[2] = vRGB[0] * 0.013929122 + vRGB[1] * 0.097097002 + vRGB[2] * 0.714185470;
}

void CLAB::__convertXYZ2LAB(std::array<float, 3>& vXYZ, std::array<float, 3>& voLAB)
{
    float RefferenceX = 0.964221f;
    float RefferenceY = 1.0f;
    float RefferenceZ = 0.825211f;
    vXYZ[0] /= RefferenceX;
    vXYZ[1] /= RefferenceY;
    vXYZ[2] /= RefferenceZ;
    for (std::uint32_t i = 0; i < vXYZ.size(); i++)
    {
        vXYZ[i] = (vXYZ[i] > 0.008856f) ? std::powf(vXYZ[i], 1 / 3.0f) : ((7.787f * vXYZ[i]) + (16 / 116.0f));
    }

    voLAB[0] = 116.0f * vXYZ[1] - 16.0f;
    voLAB[0] = (voLAB[0] > 0.0f) ? voLAB[0] : 0.0f;
    voLAB[1] = 500.0f * (vXYZ[0] - vXYZ[1]);
    voLAB[2] = 200.0f * (vXYZ[1] - vXYZ[2]);
}

void CLAB::__correctingGamma(std::array<float, 3>& vioRGB)
{
    for (std::uint32_t i = 0; i < vioRGB.size(); i++)
        vioRGB[i] = (vioRGB[i] > 0.04045) ? (std::powf(((vioRGB[i] + 0.055) / 1.055), 2.4)) : (vioRGB[i] / 12.92);
}
