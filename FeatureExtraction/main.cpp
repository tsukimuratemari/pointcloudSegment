#include "pch.h"
#include "LAB.h"
#include "PFH.h"
#include "LCCP.h"
#include "Supervoxel.h"
#include "Common.h"
#include "Direction.h"
#include "LocalDensity.h"
#include "SemanticCategory.h"
#include "RelativeElevation.h"
#include "FeatureBasedEigenvalue.h"
#include "ColorEntropy.h"
#include "DoN.h"
#include "Covariance.h"
#include "ZDirectionDensity.h"
int
main(int argc, char** argv)
{
    std::string TestPointCloudPath = "E:\\PointCloudDataSet\\sum\\2\\SubSample-InitialSegmentationResult.txt";
    PointCloudT::Ptr Cloud(new PointCloudT);
    loadCloud(TestPointCloudPath, Cloud);
    std::cout << TestPointCloudPath << std::endl;
    std::cout << "µãÔÆÊýÄ¿" << Cloud->size() << std::endl;

    //extract DoN
    DoNInputPointCloudType::Ptr pPointCloud = std::make_shared<DoNInputPointCloudType>();
    copyPointCloud(*Cloud, *pPointCloud);
    CDoN DoNExtration(pPointCloud);
    double SmallScale = 0.5f, LargeScale = 5.0f;
    std::unordered_map<int, float> DoNFeatures;
    DoNExtration.computeDoN(SmallScale, LargeScale, DoNFeatures);
    
    std::string Output = "E:\\PointCloudDataSet\\sum\\2\\Cov.txt";
    std::ofstream Writer(Output);
    Writer.setf(std::ios::fixed, std::ios::floatfield);
    Writer.precision(3);
    for (int i = 0; i < Cloud->points.size(); i++)
    {
        PointT Point = Cloud->points.at(i);

        Writer << static_cast<std::float_t>(Point.x) << " ";
        Writer << static_cast<std::float_t>(Point.y) << " ";
        Writer << static_cast<std::float_t>(Point.z) << " ";
        Writer << static_cast<std::uint32_t>(Point.r) << " ";
        Writer << static_cast<std::uint32_t>(Point.g) << " ";
        Writer << static_cast<std::uint32_t>(Point.b) << " ";
        //for (int k = 0; k < CovFeature.at(0).size(); k++)
        Writer << static_cast<std::float_t>(DoNFeatures.at(i)) << " ";
        Writer << std::endl;
    }
    Writer.close();



    //extract ColorEntropy
	/*CColorEntropy ColorEntropyExtrator(Cloud);
	float Radius = 0.7f;
	std::unordered_map<int, float> ColorEntropy;
	ColorEntropyExtrator.compute(ColorEntropy, Radius);
	std::string Output = "E:\\PointCloudDataSet\\gt_Experiment\\Color_Feat_0.7.txt";
	writeSingleFeature2TXT(Output, Cloud, ColorEntropy);*/

    //extract Covariance
    //CCovariance CovarianceExtrator(Cloud);
    //float Radius = 2.0f;
    //std::unordered_map<int, std::vector<float>>CovFeature;
    //CovarianceExtrator.computeCovariance(Radius, CovFeature);
    //std::string Output = "E:\\PointCloudDataSet\\CovDebug.txt";
    //writeSingleFeature2TXT(Output, Cloud, CovFeature);
    //extract ZDirection
    //CZDirectionDensity ZDensity(Cloud);
    //std::unordered_map<int, float> ZDensityFeature;
    //float Radius = 1.0f;
    //ZDensity.computeZDirectionDensity(Radius, ZDensityFeature);




    ////extract Supervoxel
    //float VoxelResolution = 0.0001f;
    //float SeedResolution = 0.0001f;
    //CSuperVoxel SuperVoxelor(VoxelResolution, SeedResolution);
    //SuperVoxelor.setCloud(Cloud);
    //std::map <std::uint32_t, pcl::Supervoxel<PointT>::Ptr > Supervoxels;
    //SuperVoxelor.extract(Supervoxels);
    ////SuperVoxelor.createReLabeledSuperVoxel();

    ////LCCP segmentation
    //LCCP LCCPSegmentor;
    //std::multimap<std::uint32_t, std::uint32_t> SupervoxelAdjacency;
    //SuperVoxelor.dumpSupervoxelAdjacency(SupervoxelAdjacency);
    //LCCPSegmentor.process(Supervoxels, SupervoxelAdjacency, VoxelResolution, SeedResolution);
    //pcl::LCCPSegmentation<PointT>::SupervoxelAdjacencyList SupervoxelConvexityAdjacencyList;
    //LCCPSegmentor.getConcaveAndConvexAdjacencyList(SupervoxelConvexityAdjacencyList);

    ////extract eigenvalue-based features
    //CFeatureBasedEigenvalue FeatureBasedEigenvaluer;
    //std::map<std::uint32_t, SFeaturesBasedEigenvalue> SVFeatureBasedEigenvalues;
    //FeatureBasedEigenvaluer.compute(SupervoxelConvexityAdjacencyList, Supervoxels, SVFeatureBasedEigenvalues);

    ////extract relative elevation
    //std::uint32_t CellScale = 10;
    //std::uint32_t KernelScale = 5;
    //CRelativeElevation RelativeElevationOperator(Cloud, CellScale, KernelScale);
    //std::map<std::uint32_t, float> SVRelativeElevations;
    //RelativeElevationOperator.compute(Supervoxels, SVRelativeElevations);

    ////extract Direction
    //CSupervoxelDirection SVDirectioner;
    //std::map<std::uint32_t, double> SVDirections;
    //SVDirectioner.compute(Supervoxels, SVDirections);

    ////extract LAB
    //CLAB LABer;
    //std::map<std::uint32_t, std::array<float, 3>> SVLABs;
    //LABer.computeBasedSV(Supervoxels, SVLABs);

    ////extract PFH
    //CPFH PFHer;
    //std::map<std::uint32_t, Eigen::VectorXf> SVPFHs;
    //PFHer.compute(Cloud, Supervoxels, SVPFHs);

    ////extract Local Density
    //CLocalDensity LocalDensityor;
    //std::map<std::uint32_t, double> SVLocalDensitys;
    //LocalDensityor.compute(Supervoxels, SVLocalDensitys);

    ////compute SemanticCategory
    //CSemanticCategory SemanticCategoryor;
    //SemanticCategoryor.compute(Cloud, Supervoxels);

    //std::map<std::uint32_t, SSupervoxelVFeatures> SVFeatureList;
    //for (auto& SV : Supervoxels)
    //{
    //    SSupervoxelVFeatures SVFeatures;
    //    SVFeatures._Label = SV.first;
    //    SVFeatures._LAB = SVLABs.at(SV.first);
    //    SVFeatures._PFH = SVPFHs.at(SV.first);
    //    SVFeatures._Direction = SVDirections.at(SV.first);
    //    SVFeatures._LocalDensity = SVLocalDensitys.at(SV.first);
    //    SVFeatures._SemanticCategory = SV.second->centroid_.a;
    //    SVFeatures._RelativeElevation = SVRelativeElevations.at(SV.first);
    //    SVFeatures._FeaturesBasedEigenvalue = SVFeatureBasedEigenvalues.at(SV.first);
    //    SVFeatureList.insert(std::make_pair(SV.first, SVFeatures));
    //}
    //writeFeature2TXT(SVFeatureList,"Feature.txt");
    //writeFeatureWithPoint2TXT(Supervoxels, SVFeatureList, argv[2]);
    //writeSupervoxelWithPointLabel2TXT(Supervoxels, "SupervoxelWithPointLabel.txt");
    //writeSupervoxelWithCentroidLabel2TXT(Supervoxels, "SupervoxelWithCentroidLabel.txt");
    //return (0);
}
