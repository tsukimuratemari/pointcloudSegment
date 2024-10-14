#include "pch.h"
#include "Supervoxel.h"
#include "SemanticCategory.h"

int
main(int argc, char** argv)
{
    for (int i = 1; i < 2; i++)
    {

        std::string InputFileName = "E:/PointCloudDataSet/Experiment/multivoxel/origin/all/origin.txt";
        std::string OutputFileName = "E:/PointCloudDataSet/Experiment/multivoxel/temp/output" + std::to_string(i-1) + ".txt";
        std::cout << InputFileName << std::endl;
        PointCloudT::Ptr Cloud(new PointCloudT);
        loadCloud(InputFileName, Cloud);
        std::cout << "µãÔÆÊýÄ¿" << Cloud->size() << std::endl;

        //extract Supervoxel
        float VoxelResolution = 0.005f;
        float SeedResolution = 0.01f;
        CSuperVoxel SuperVoxelor(VoxelResolution, SeedResolution);
        SuperVoxelor.setCloud(Cloud);
        std::map <std::uint32_t, pcl::Supervoxel<PointT>::Ptr > Supervoxels;
        SuperVoxelor.extract(Supervoxels);
        SuperVoxelor.createReLabeledSuperVoxel();

        //compute SemanticCategory
        CSemanticCategory SemanticCategoryor;
        SemanticCategoryor.compute(Cloud, Supervoxels);
        loadClassifyResultAndComputeAcc(OutputFileName, Supervoxels);
    }
    return 0;
}