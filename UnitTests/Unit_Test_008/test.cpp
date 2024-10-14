#include "pch.h"
#include "Common.h"
#include "ElevationDifference.h"
#include "SizeOfElevationDifferenceCluster.h"

std::string TESTDATADIR = "C:\\Users\\HPJ\\Desktop\\Paper\\Exp\\Scene4\\subsample_cloud\\Randlanet\\";
std::string CloudPath = TESTDATADIR + "SubSample_mapd_Label_Randlanet_block_0_2.txt";
std::string FeaturePath = TESTDATADIR + "Feature_SizeOfElevationDifferenceCluster.txt";

TEST(SizeOfElevationDifferenceCluster, NT_Run) {
  PointCloudT::Ptr Cloud= std::make_shared<PointCloudT>();
  loadCloud(CloudPath, Cloud);

  CElevationDifference EDr(Cloud);
  std::unordered_map<int, float> ElevationDifferences;
  EDr.compute(ElevationDifferences);

  CSizeOfElevationDifferenceCluster FeatureOperator(Cloud, ElevationDifferences);
  std::unordered_map<PointSerialNumber, FeatureValue> PointMapFeature;
  FeatureOperator.compute(PointMapFeature);

  EXPECT_EQ(Cloud->size(), PointMapFeature.size());

  auto writeFeature = [&](const std::string& vPath)
  {
	  std::ofstream Writer(vPath);
	  for (int i = 0; i < Cloud->size(); i++)
	  {
		  Writer << (double)Cloud->points.at(i).x << " ";
		  Writer << (double)Cloud->points.at(i).y << " ";
		  Writer << (double)Cloud->points.at(i).z << " ";
		  Writer << (int)Cloud->points.at(i).r << " ";
		  Writer << (int)Cloud->points.at(i).g << " ";
		  Writer << (int)Cloud->points.at(i).b << " ";
		  Writer << ElevationDifferences.at(i) << " ";
		  Writer << PointMapFeature.at(i) << " ";
		  Writer << "\n";
	  }
  };

  writeFeature(FeaturePath);
}