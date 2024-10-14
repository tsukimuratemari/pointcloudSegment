#include "pch.h"
#include "ColorEntropy.h"

CColorEntropy::CColorEntropy(const PointCloudT::Ptr& vCloud)
{
	m_pCloud = std::make_shared<PointCloudT>();
	m_pSearchTree = std::make_shared<pcl::KdTreeFLANN<PointT>>();
	m_pCloud = vCloud;
	m_pSearchTree->setInputCloud(m_pCloud);
}

void CColorEntropy::compute(std::unordered_map<int, float>& voColorEntroys, float viNeighbourhoodRadius)
{
	voColorEntroys.clear();
	for (int i = 0; i < m_pCloud->points.size(); i++)
	{
		std::vector<int> PointRadiusSearchIndex;
		std::vector<float> PointRadiusSquaredDistance;
		if (m_pSearchTree->radiusSearch(m_pCloud->points[i], viNeighbourhoodRadius, PointRadiusSearchIndex, PointRadiusSquaredDistance) == 0)
			PointRadiusSearchIndex.push_back(i);

		voColorEntroys.insert(std::pair(i, __computeColorVarience(PointRadiusSearchIndex)));
	}
	__normalizeColorEntropy(voColorEntroys);
}

void CColorEntropy::__normalizeColorEntropy(std::unordered_map<int, float>& vioColorEntropy)
{
	float NormalizeMax = 1.0f, NormalizeMin = 0.0f;
	float Max = FLT_MIN, Min = FLT_MAX;
	for (int i = 0; i < vioColorEntropy.size(); i++)
	{
		if (vioColorEntropy.at(i) > Max) Max = vioColorEntropy.at(i);
		if (vioColorEntropy.at(i) < Min) Min = vioColorEntropy.at(i);
	}

	for (int i = 0; i < vioColorEntropy.size(); i++)
	{
		vioColorEntropy.at(i) = (NormalizeMax - NormalizeMin) * (vioColorEntropy.at(i) - Min) / (Max - Min + 1e-8) + NormalizeMin;
	}
}

float CColorEntropy::__computeColorVarience(std::vector<int>& viPointIndex)
{
	SRGBColor AverageColor;
	for (int i = 0; i < viPointIndex.size(); i++)
	{
		AverageColor.R+= m_pCloud->points[viPointIndex[i]].r;
		AverageColor.G += m_pCloud->points[viPointIndex[i]].g;
		AverageColor.B += m_pCloud->points[viPointIndex[i]].b;
	}
	AverageColor.R /= viPointIndex.size();
	AverageColor.G /= viPointIndex.size();
	AverageColor.B /= viPointIndex.size();

	SRGBColor VarienceColor;
	for (int i = 0; i < viPointIndex.size(); i++)
	{
		VarienceColor.R += pow(m_pCloud->points[viPointIndex[i]].r - AverageColor.R, 2);
		VarienceColor.G += pow(m_pCloud->points[viPointIndex[i]].g - AverageColor.G, 2);
		VarienceColor.B += pow(m_pCloud->points[viPointIndex[i]].b - AverageColor.B, 2);
	}
	VarienceColor.R /= (viPointIndex.size() - 1);
	VarienceColor.G /= (viPointIndex.size() - 1);
	VarienceColor.B /= (viPointIndex.size() - 1);

	//float ColoarVarience = VarienceColor.R + VarienceColor.G + VarienceColor.B;
	float ColoarVarience = VarienceColor.R + VarienceColor.B;
	return ColoarVarience;
}