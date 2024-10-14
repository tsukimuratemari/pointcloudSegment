#pragma once
#include"Common.h"
#include<Eigen/Core>

class CProjectMap;
class CProjectMapGenerator;

class CProjectionElevation
{
public:
	CProjectionElevation(const PointCloudT::Ptr& vCloud, int vWidth, int vHeight) :m_pCloud(vCloud)
	{
		if (!__initializeProjectMap(vWidth, vHeight)) { _ASSERTE(false, "ProjectMap's initialization was failed!"); }
	};
	~CProjectionElevation() = default;

	void compute(std::unordered_map<int, float>& voProjectElevations);
	_NODISCARD bool setMaxAllowedDistanceWithinACluster(float vDistance);

#ifdef _UNIT_TEST
	std::vector<float> computeMaxEvelationOfClusters(const PointCloudT& vCloud)
	{
		return __computeMaxEvelationOfClusters(vCloud);
	}
#endif // _UNIT_TEST


private:
	_NODISCARD std::vector<float> __computeMaxEvelationOfClusters(const PointCloudT& vCloud);
	_NODISCARD bool __initializeProjectMap(int vWidth, int vHeight);
	_NODISCARD float __sampleProjectElevation(const PointT& vPoint) const;
	void __generateElevationMap();
	
private:
	std::shared_ptr<CProjectMapGenerator> m_pCloudMapGenerator;
	Eigen::Matrix<std::vector<float>, -1, -1> m_ElevationMap;
	float m_MaxAllowedDistanceWithinACluster = 3.0f;
	std::shared_ptr<CProjectMap> m_pCloudMap;
	float m_EmptyValue = -FLT_MAX;
	PointCloudT::Ptr m_pCloud;
};








