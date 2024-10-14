#pragma once
#include<vector>
#include<map>
#include<set>
#include"Common.h"

class CPostProcessing {
public:
    CPostProcessing(const ESemanticCategory vSemanticLabel,
        std::vector<std::vector<double>>& vSVs,
        const std::multimap<std::uint32_t, std::uint32_t>& vSVAdjacencyList) :m_SemanticLabel(vSemanticLabel),
        m_OriginFormatSVs(vSVs),m_SVAdjacencyList(vSVAdjacencyList){
        _convertDataFormat2Map(vSVs);
        _convertDataFormat2MultiMap(vSVs);
    };
    virtual ~CPostProcessing() = default;

    virtual void process()=0;
    auto getResult() const { return m_OriginFormatSVs; }

protected:
    float _computeClusterHeight(const std::set<std::uint32_t>& vCluster) const;
    std::uint32_t _computeArea(const std::set<std::uint32_t>& vCluster) const;
    void _changeLabelBasedMaxCountLabelInNeighbors(const std::set<std::uint32_t>& vCluster, const std::set<std::uint32_t>& vNeighbors);
    void _clustering(std::vector<std::set<std::uint32_t>>& voClusters) const;
	void _convertDataFormat2Map(const std::vector<std::vector<double>>& vSVs);
	void _convertDataFormat2MultiMap(const std::vector<std::vector<double>>& vSVs);
	void _convertMap2DoubleVector();
	std::set<std::uint32_t> _findClusterNeighbourHood(const std::set<std::uint32_t>& vCluster) const;
	ESemanticCategory _findMaxCountLabelOfNeighbourHood(const std::set<std::uint32_t>& vNeighbourHood) const;
    

    ESemanticCategory m_SemanticLabel;
    std::map<std::uint32_t, std::vector<double>> m_Supervoxels;
    std::multimap<std::uint32_t, std::vector<double>> m_Points;
    std::multimap<std::uint32_t, std::uint32_t> m_SVAdjacencyList;
    std::uint32_t m_HeightColumnIndex = 2;
    std::uint32_t m_SemanticLabelColumnIndex = 7;
    std::uint32_t m_SupervoxelSerialNumColumnIndex = 6;
    std::vector<std::vector<double>> m_OriginFormatSVs;
};

class CGroundPostProcessing :public CPostProcessing
{
public:
    CGroundPostProcessing(const ESemanticCategory vSemanticLabel,
        std::vector<std::vector<double>>& vSVs,
        const std::multimap<std::uint32_t, std::uint32_t>& vSVAdjacencyList):CPostProcessing(vSemanticLabel, vSVs, vSVAdjacencyList) {};
    ~CGroundPostProcessing()=default;

    virtual void process() override;

private:
    bool __ruleByGroundArea(const std::set<std::uint32_t>& vGroundCluster) const;
    bool __ruleByNeighbourHeight(const std::set<std::uint32_t>& vCluster, const std::set<std::uint32_t>& vNeighbors) const;

    std::uint32_t m_MinGroundArea = 100;
};

class CVehiclesPostProcessing :public CPostProcessing {
public:
    CVehiclesPostProcessing(const ESemanticCategory vSemanticLabel,
        std::vector<std::vector<double>>& vSVs,
        const std::multimap<std::uint32_t, std::uint32_t>& vSVAdjacencyList) :CPostProcessing(vSemanticLabel,vSVs,vSVAdjacencyList) {};
    ~CVehiclesPostProcessing() = default;

    virtual void process() override;

private:
    bool __ruleByVechicleArea(const std::set<std::uint32_t>& vVehicleCluster) const;
    bool __ruleByNeighbour(const std::set<std::uint32_t>& vNeighbors) const;
    bool __ruleByRelativeHeight(const std::uint32_t vVehicleSV, std::set<std::uint32_t> vNeighbors) const;

    std::uint32_t m_MinVechicleArea = 300;
    float m_MinHeightDifference = 0.00375;
};

class CBuildingPostProcessing :public CPostProcessing {
public:
    CBuildingPostProcessing(const ESemanticCategory vSemanticLabel,
        std::vector<std::vector<double>>& vSVs,
        const std::multimap<std::uint32_t, std::uint32_t>& vSVAdjacencyList) :CPostProcessing(vSemanticLabel, vSVs, vSVAdjacencyList) {};
    ~CBuildingPostProcessing() = default;

    virtual void process() override;

private:
    bool __ruleByBuildingArea(const std::set<std::uint32_t>& vBuildingCluster) const;
    bool __ruleByNeighborHeight(const std::set<std::uint32_t>& vBuildingCluster, const std::set<std::uint32_t>& vNeighborsExceptTree) const;

	float m_HeightDifferenceThreshold = 0;
    std::uint32_t m_MinBuildingArea = 200;
};

class CTreePostProcessing :public CPostProcessing {
public:
    CTreePostProcessing(const ESemanticCategory vSemanticLabel,
        std::vector<std::vector<double>>& vSVs,
        const std::multimap<std::uint32_t, std::uint32_t>& vSVAdjacencyList) :CPostProcessing(vSemanticLabel, vSVs, vSVAdjacencyList) {};
    ~CTreePostProcessing() = default;

    virtual void process() override;

private:
	bool __ruleByTreeArea(const std::set<std::uint32_t>& vTreeCluster) const;
    bool __ruleByNormalDifference(const std::set<std::uint32_t>& vTreeCluster) const;

    std::uint32_t m_MinTreeArea = 200;
};

