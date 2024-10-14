#pragma once

template <typename FeatureType>
class CPointCloudHistogram
{
public:
	CPointCloudHistogram(int viIntervalNum, std::vector<std::vector<FeatureType>>& viFeature);


	~CPointCloudHistogram() = default;

	std::vector<float> computeHistogram(std::vector<FeatureType>& viFeature);


	
private:

	int m_IntervalNum;

	std::vector<float> m_IntervalScale;

	std::vector<std::vector<float>> m_PointCloudIntervalDistribution;
};

