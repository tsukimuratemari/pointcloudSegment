#include "pch.h"
#include "PointCloud.h"

template class CPointCloudHistogram<int>;
template class CPointCloudHistogram<float>;

template <typename FeatureType>
CPointCloudHistogram<FeatureType>::CPointCloudHistogram(int viIntervalNum, std::vector<std::vector<FeatureType>>& viFeature)
{
	m_IntervalNum = viIntervalNum;
	FeatureType MinFeature = INT32_MAX, MaxFeature = INT32_MIN;
	for (int i = 0; i < viFeature.size(); i++)
	{
		std::sort(viFeature[i].begin(), viFeature[i].end());
		if (MinFeature > viFeature[i].front())MinFeature = viFeature[i].front();
		if (MaxFeature < viFeature[i].back())MaxFeature = viFeature[i].back();
	}
	float IntervalScale = float(MaxFeature - MinFeature) / viIntervalNum;

	for (int i = 1; i < viIntervalNum; i++)
	{
		m_IntervalScale.push_back(MinFeature + i * IntervalScale);
	}

	for (int i = 0; i < viFeature.size(); i++)
	{
		std::vector<int> IntervalDistributionNum(viIntervalNum, 0);

		for (int k = 0; k < viFeature[i].size(); k++)
		{
			if (m_IntervalScale[viIntervalNum - 2] <= viFeature[i][k])
			{
				IntervalDistributionNum[viIntervalNum - 1]++;
				continue;
			}

			for (int m = 0; m < m_IntervalScale.size(); m++)
			{
				if (m_IntervalScale[m] > viFeature[i][k])
				{
					IntervalDistributionNum[m]++;
					break;
				}
			}
		}
		std::vector<float> IntervalDistribution;
		//int test = 0;
		for (int k = 0; k < m_IntervalNum; k++)
		{
			//test += IntervalDistributionNum[k];
			IntervalDistribution.push_back(float(IntervalDistributionNum[k]) / viFeature[i].size());
		}
		m_PointCloudIntervalDistribution.push_back(IntervalDistribution);
	}
};

template <typename FeatureType>
std::vector<float> CPointCloudHistogram<FeatureType>::computeHistogram(std::vector<FeatureType>& viFeature)
{
	std::vector<int> IntervalDistributionNum(m_IntervalNum, 0);
	for (int i = 0; i < viFeature.size(); i++)
	{
		if (m_IntervalScale[m_IntervalNum - 2] <= viFeature[i])
		{
			IntervalDistributionNum[m_IntervalNum - 1]++;
			continue;
		}
		for (int k = 0; k < m_IntervalScale.size(); k++)
		{
			if (m_IntervalScale[k] > viFeature[i])
			{
				IntervalDistributionNum[k]++;
				break;
			}
		}
	}

	std::vector<float> IntervalDistribution;
	for (int i = 0; i < m_IntervalNum; i++)
	{
		IntervalDistribution.push_back(float(IntervalDistributionNum[i]) / viFeature.size());
	}
	std::vector<float> BhattacharyyaCoefficientVector;
	for (int i = 0; i < m_PointCloudIntervalDistribution.size(); i++)
	{
		float BhattacharyyaCoefficient = 0.0f;
		for (int k = 0; k < m_PointCloudIntervalDistribution[i].size(); k++)
		{
			BhattacharyyaCoefficient += sqrt(IntervalDistribution[k] * m_PointCloudIntervalDistribution[i][k]);
		}
		BhattacharyyaCoefficientVector.push_back(BhattacharyyaCoefficient);
	}

	return BhattacharyyaCoefficientVector;
};
