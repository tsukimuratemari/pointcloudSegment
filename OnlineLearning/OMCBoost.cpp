// -*- C++ -*-
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * Written (W) 2010 Amir Saffari, amir@ymer.org
 * Copyright (C) 2010 Amir Saffari,
 *                    Institute for Computer Graphics and Vision,
 *                    Graz University of Technology, Austria
 */

#include <cstdlib>
#include <iostream>
#include <string>
#include <string.h>
#include <libconfig.h++>

#include "data.h"
#include "Utils.h"
#include "experimenter.h"
#include "online_rf.h"
#include "linear_larank.h"
#include "online_mcboost.h"
#include "online_mclpboost.h"

using namespace std;
using namespace libconfig;

typedef enum {
    ORT, ORF, OMCBOOST, OMCLPBOOST, LARANK
} CLASSIFIER_TYPE;

//! Prints the interface help message
void help() {
    cout << endl;
    cout << "OMCBoost Classification Package:" << endl;
    cout << "Input arguments:" << endl;
    cout << "\t -h | --help : \t will display this message." << endl;
    cout << "\t -c : \t\t path to the config file." << endl << endl;
    cout << "\t --ort : \t use Online Random Tree (ORT) algorithm." << endl;
    cout << "\t --orf : \t use Online Random Forest (ORF) algorithm." << endl;
    cout << "\t --omcb : \t use Online MCBoost algorithm." << endl;
    cout << "\t --omclp : \t use Online MCLPBoost algorithm." << endl;
    cout << "\t --omclppd : \t use Online MCLPBoostPD algorithm." << endl;
    cout << "\t --larank : \t use Online LaRank algorithm." << endl;
    cout << endl << endl;
    cout << "\t --train : \t train the classifier." << endl;
    cout << "\t --test : \t test the classifier." << endl;
    cout << "\t --t2 : \t train and test the classifier at the same time." << endl;
    cout << endl << endl;
    cout << "\tExamples:" << endl;
    cout << "\t ./OMCBoost -c conf/orf.conf --orf --train --test" << endl;
}




int main(int argc, char* argv[])
{
   // CPointCloudTool PointCloudTool;
   // vector<vector<double>> OriginData;
   // string DLResultFilePath = "E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\train.txt";
   // PointCloudTool.loadOriginArray(DLResultFilePath, OriginData);
   // vector<vector<double>> DLTrainData, ManualTrainData;
   // std::string ConfFilePath = "conf/omcb.conf";
   // Hyperparameters HP(ConfFilePath);
   // DataSet TestData(OriginData);
   // vector<string> PosManualResultFilePathVector;
   // vector<string> NegManualResultFilePathVector;
   // ESemanticCategory CurrentSemanticLabel;
   // 
   // /*
   // vector<vector<double>> ManualData1;
   // PointCloudTool.loadOriginArray("E:/PointCloudDataSet/Experiment/omcboost/Manual_1.txt", ManualData1);
   // DataSet DLTrainDataSet(OriginData);
   // DataSet ManualDLTrainData(ManualData1);

   // Classifier* Model = new OnlineMCBoost(HP, DLTrainDataSet.m_numClasses, DLTrainDataSet.m_numFeatures, DLTrainDataSet.m_minFeatRange, DLTrainDataSet.m_maxFeatRange);
   // train(Model, DLTrainDataSet, HP);
   // train(Model, ManualDLTrainData, HP);
   // vector<Result> DLResults = test(Model, TestData, HP);
   // PointCloudTool.visualizeORFApproachDLResult(OriginData, DLResults);
   // PointCloudTool.saveArrayFile("E:/PointCloudDataSet/Experiment/omcboost/Manual1_Result.txt", OriginData);
   // */
   // 
   // ////*****************************************************************
   // ////Building
   // ////DLResult approaching stage
   // //DLTrainData = OriginData;
   // //CurrentSemanticLabel = ESemanticCategory::Building;
   // //PointCloudTool.binarizeDLTrainData(CurrentSemanticLabel, DLTrainData);
   // //DataSet BuildingDLTrainData(DLTrainData);
   // //Classifier* BuildingModel = new OnlineRF(HP, BuildingDLTrainData.m_numClasses, BuildingDLTrainData.m_numFeatures, BuildingDLTrainData.m_minFeatRange, BuildingDLTrainData.m_maxFeatRange);
   // //train(BuildingModel, BuildingDLTrainData, HP);
   // //vector<Result> BuildingResults = test(BuildingModel, TestData, HP);
   // //PointCloudTool.visualizeORFApproachDLResult(DLTrainData, BuildingResults);
   // //PointCloudTool.saveArrayFile("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Building_ApproachDLResult.txt", DLTrainData);
   // ////ManualResult approaching stage 1
   // //PosManualResultFilePathVector.push_back("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Building_Positive_1.txt");
   // //NegManualResultFilePathVector.push_back("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Building_Negative_1.txt");
   // //PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
   // //DataSet BuildingManualTrainData_1(ManualTrainData);
   // //train(BuildingModel, BuildingManualTrainData_1, HP);
   // //BuildingResults.clear();
   // //BuildingResults = test(BuildingModel, TestData, HP);
   // //PointCloudTool.visualizeORFApproachManualResult(OriginData, CurrentSemanticLabel, BuildingResults);
   // //PointCloudTool.saveArrayFile("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Building_ManualResult_1.txt", OriginData);

   // ////ManualResult approaching stage 2
   // //PosManualResultFilePathVector.push_back("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Building_Positive_2.txt");
   // //NegManualResultFilePathVector.push_back("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Building_Negative_2.txt");
   // //PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
   // //DataSet BuildingManualTrainData_2(ManualTrainData);
   // //train(BuildingModel, BuildingManualTrainData_2, HP);
   // //BuildingResults.clear();
   // //BuildingResults = test(BuildingModel, TestData, HP);
   // //PointCloudTool.visualizeORFApproachManualResult(OriginData, CurrentSemanticLabel, BuildingResults);
   // //PointCloudTool.saveArrayFile("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Building_ManualResult_2.txt", OriginData);

   // ////ManualResult approaching stage 3
   // //PosManualResultFilePathVector.push_back("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Building_Positive_3.txt");
   // //NegManualResultFilePathVector.push_back("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Building_Negative_3.txt");
   // //PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
   // //DataSet BuildingManualTrainData_3(ManualTrainData);
   // //train(BuildingModel, BuildingManualTrainData_3, HP);
   // //BuildingResults.clear();
   // //BuildingResults = test(BuildingModel, TestData, HP);
   // //PointCloudTool.visualizeORFApproachManualResult(OriginData, CurrentSemanticLabel, BuildingResults);
   // //PointCloudTool.saveArrayFile("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Building_ManualResult_3.txt", OriginData);


   // //PosManualResultFilePathVector.clear();
   // //NegManualResultFilePathVector.clear();
   // //delete BuildingModel;


   // //*****************************************************************
   // //Car
   // //DLResult approaching stage
   // DLTrainData = OriginData;
   // CurrentSemanticLabel = ESemanticCategory::Vehicle;
   // PointCloudTool.binarizeDLTrainData(CurrentSemanticLabel, DLTrainData);
   // DataSet VehicleDLTrainData(DLTrainData);
   // vector<Result> VehicleResults;
   // Classifier* VehicleModel = new OnlineRF(HP, VehicleDLTrainData.m_numClasses, VehicleDLTrainData.m_numFeatures, VehicleDLTrainData.m_minFeatRange, VehicleDLTrainData.m_maxFeatRange);
   // train(VehicleModel, VehicleDLTrainData, HP);
   // VehicleResults = test(VehicleModel, TestData, HP);
   // PointCloudTool.visualizeORFApproachDLResult(DLTrainData, VehicleResults);
   // PointCloudTool.saveArrayFile("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Vehicle_ApproachDLResult.txt", DLTrainData);
   // //ManualResult approaching stage 1
   // PosManualResultFilePathVector.push_back("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Vehicle_Positive_1.txt");
   // NegManualResultFilePathVector.push_back("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Vehicle_Negative_1.txt");
   // PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
   // DataSet VehicleManualTrainData_1(ManualTrainData);
   // train(VehicleModel, VehicleManualTrainData_1, HP);
   // VehicleResults.clear();
   // VehicleResults = test(VehicleModel, TestData, HP);
   // PointCloudTool.visualizeORFApproachManualResult(OriginData, CurrentSemanticLabel, VehicleResults);
   // PointCloudTool.saveArrayFile("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Vehicle_ManualResult_1.txt", OriginData);
   // PosManualResultFilePathVector.clear();
   // NegManualResultFilePathVector.clear();
   // //ManualResult approaching stage 2
   // PosManualResultFilePathVector.push_back("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Vehicle_Positive_2.txt");
   // NegManualResultFilePathVector.push_back("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Vehicle_Negative_2.txt");
   // PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
   // DataSet VehicleManualTrainData_2(ManualTrainData);
   // train(VehicleModel, VehicleManualTrainData_2, HP);
   // VehicleResults.clear();
   // VehicleResults = test(VehicleModel, TestData, HP);
   // PointCloudTool.visualizeORFApproachManualResult(OriginData, CurrentSemanticLabel, VehicleResults);
   // PointCloudTool.saveArrayFile("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Vehicle_ManualResult_2.txt", OriginData);
   // //ManualResult approaching stage 3
   // PosManualResultFilePathVector.push_back("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Vehicle_Positive_3.txt");
   // NegManualResultFilePathVector.push_back("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Vehicle_Negative_3.txt");
   // PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
   // DataSet VehicleManualTrainData_3(ManualTrainData);
   // train(VehicleModel, VehicleManualTrainData_3, HP);
   // VehicleResults.clear();
   // VehicleResults = test(VehicleModel, TestData, HP);
   // PointCloudTool.visualizeORFApproachManualResult(OriginData, CurrentSemanticLabel, VehicleResults);
   // PointCloudTool.saveArrayFile("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Vehicle_ManualResult_3.txt", OriginData);

   // PosManualResultFilePathVector.clear();
   // NegManualResultFilePathVector.clear();
   // delete VehicleModel;

   ////*****************************************************************
   ////Tree
   ////DLResult approaching stage
   //DLTrainData = OriginData;
   //CurrentSemanticLabel = ESemanticCategory::Tree;
   //vector<Result> TreeResults;
   //PointCloudTool.binarizeDLTrainData(CurrentSemanticLabel, DLTrainData);
   //DataSet TreeDLTrainData(DLTrainData);
   //Classifier* TreeModel = new OnlineRF(HP, TreeDLTrainData.m_numClasses, TreeDLTrainData.m_numFeatures, TreeDLTrainData.m_minFeatRange, TreeDLTrainData.m_maxFeatRange);
   //train(TreeModel, TreeDLTrainData, HP);
   //TreeResults = test(TreeModel, TestData, HP);
   //PointCloudTool.visualizeORFApproachDLResult(DLTrainData, TreeResults);
   //PointCloudTool.saveArrayFile("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Tree_ApproachDLResult.txt", DLTrainData);
   ////ManualResult approaching stage 1
   //PosManualResultFilePathVector.push_back("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Tree_Positive_1.txt");
   //NegManualResultFilePathVector.push_back("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Tree_Negative_1.txt");
   //PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
   //DataSet TreeManualTrainData_1(ManualTrainData);
   //train(TreeModel, TreeManualTrainData_1, HP);
   //TreeResults.clear();
   //TreeResults = test(TreeModel, TestData, HP);
   //PointCloudTool.visualizeORFApproachManualResult(OriginData, CurrentSemanticLabel, TreeResults);
   //PointCloudTool.saveArrayFile("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Tree_ManualResult_1.txt", OriginData);
   //PosManualResultFilePathVector.clear();
   //NegManualResultFilePathVector.clear();
   ////ManualResult approaching stage 2
   //PosManualResultFilePathVector.push_back("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Tree_Positive_3.txt");
   //NegManualResultFilePathVector.push_back("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Tree_Negative_3.txt");
   //PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
   //DataSet TreeManualTrainData_2(ManualTrainData);
   //train(TreeModel, TreeManualTrainData_2, HP);
   //TreeResults.clear();
   //TreeResults = test(TreeModel, TestData, HP);
   //PointCloudTool.visualizeORFApproachManualResult(OriginData, CurrentSemanticLabel, TreeResults);
   //PointCloudTool.saveArrayFile("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Tree_ManualResult_2.txt", OriginData);

   //
   //PosManualResultFilePathVector.clear();
   //NegManualResultFilePathVector.clear();
   //delete TreeModel;


   // //*****************************************************************
   // //Ground
   // //DLResult approaching stage
   // DLTrainData = OriginData;
   // CurrentSemanticLabel = ESemanticCategory::Ground;
   // vector<Result> GroundResults;
   // PointCloudTool.binarizeDLTrainData(CurrentSemanticLabel, DLTrainData);
   // DataSet GroundDLTrainData(DLTrainData);
   // Classifier* GroundModel = new OnlineRF(HP, GroundDLTrainData.m_numClasses, GroundDLTrainData.m_numFeatures, GroundDLTrainData.m_minFeatRange, GroundDLTrainData.m_maxFeatRange);
   // train(GroundModel, GroundDLTrainData, HP);
   // GroundResults = test(GroundModel, TestData, HP);
   // PointCloudTool.visualizeORFApproachDLResult(DLTrainData, GroundResults);
   // PointCloudTool.saveArrayFile("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Ground_ApproachDLResult.txt", DLTrainData);
   // //ManualResult approaching stage 1
   // PosManualResultFilePathVector.push_back("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Ground_Positive_1.txt");
   // NegManualResultFilePathVector.push_back("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Ground_Negative_1.txt");
   // PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
   // DataSet GroundManualTrainData_1(ManualTrainData);
   // train(GroundModel, GroundManualTrainData_1, HP);
   // GroundResults.clear();
   // GroundResults = test(GroundModel, TestData, HP);
   // PointCloudTool.visualizeORFApproachManualResult(OriginData, CurrentSemanticLabel, GroundResults);
   // PointCloudTool.saveArrayFile("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Ground_ManualResult_1.txt", OriginData);
   // PosManualResultFilePathVector.clear();
   // NegManualResultFilePathVector.clear();

   // //ManualResult approaching stage 2
   // PosManualResultFilePathVector.push_back("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Ground_Positive_2.txt");
   // NegManualResultFilePathVector.push_back("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Ground_Negative_2.txt");
   // PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
   // DataSet GroundManualTrainData_2(ManualTrainData);
   // train(GroundModel, GroundManualTrainData_2, HP);
   // GroundResults.clear();
   // GroundResults = test(GroundModel, TestData, HP);
   // PointCloudTool.visualizeORFApproachManualResult(OriginData, CurrentSemanticLabel, GroundResults);
   // PointCloudTool.saveArrayFile("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Ground_ManualResult_2.txt", OriginData);
   // PosManualResultFilePathVector.clear();
   // NegManualResultFilePathVector.clear();
   // //ManualResult approaching stage 3
   // PosManualResultFilePathVector.push_back("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Ground_Positive_3.txt");
   // NegManualResultFilePathVector.push_back("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Ground_Negative_3.txt");
   // PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
   // DataSet GroundManualTrainData_3(ManualTrainData);
   // train(GroundModel, GroundManualTrainData_3, HP);
   // GroundResults.clear();
   // GroundResults = test(GroundModel, TestData, HP);
   // PointCloudTool.visualizeORFApproachManualResult(OriginData, CurrentSemanticLabel, GroundResults);
   // PointCloudTool.saveArrayFile("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Ground_ManualResult_3.txt", OriginData);
   // PosManualResultFilePathVector.clear();
   // NegManualResultFilePathVector.clear();
   // //ManualResult approaching stage 4
   // PosManualResultFilePathVector.push_back("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Ground_Positive_4.txt");
   // NegManualResultFilePathVector.push_back("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Ground_Negative_4.txt");
   // PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
   // DataSet GroundManualTrainData_4(ManualTrainData);
   // train(GroundModel, GroundManualTrainData_4, HP);
   // GroundResults.clear();
   // GroundResults = test(GroundModel, TestData, HP);
   // PointCloudTool.visualizeORFApproachManualResult(OriginData, CurrentSemanticLabel, GroundResults);
   // PointCloudTool.saveArrayFile("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Ground_ManualResult_4.txt", OriginData);

   // ////ManualResult approaching stage 5
   // //PosManualResultFilePathVector.push_back("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Ground_Positive_5.txt");
   // //NegManualResultFilePathVector.push_back("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Ground_Negative_5.txt");
   // //PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
   // //DataSet GroundManualTrainData_5(ManualTrainData);
   // //train(GroundModel, GroundManualTrainData_5, HP);
   // //GroundResults.clear();
   // //GroundResults = test(GroundModel, TestData, HP);
   // //PointCloudTool.visualizeORFApproachManualResult(OriginData, CurrentSemanticLabel, GroundResults);
   // //PointCloudTool.saveArrayFile("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Ground_ManualResult_5.txt", OriginData);

   // ////ManualResult approaching stage 6
   // //PosManualResultFilePathVector.push_back("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Ground_Positive_6.txt");
   // //NegManualResultFilePathVector.push_back("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Ground_Negative_6.txt");
   // //PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
   // //DataSet GroundManualTrainData_6(ManualTrainData);
   // //train(GroundModel, GroundManualTrainData_6, HP);
   // //GroundResults.clear();
   // //GroundResults = test(GroundModel, TestData, HP);
   // //PointCloudTool.visualizeORFApproachManualResult(OriginData, CurrentSemanticLabel, GroundResults);
   // //PointCloudTool.saveArrayFile("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Ground_ManualResult_6.txt", OriginData);

   // ////ManualResult approaching stage 7
   // //PosManualResultFilePathVector.push_back("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Ground_Positive_7.txt");
   // //NegManualResultFilePathVector.push_back("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Ground_Negative_7.txt");
   // //PointCloudTool.mergeORFManualResult(PosManualResultFilePathVector, NegManualResultFilePathVector, ManualTrainData);
   // //DataSet GroundManualTrainData_7(ManualTrainData);
   // //train(GroundModel, GroundManualTrainData_7, HP);
   // //GroundResults.clear();
   // //GroundResults = test(GroundModel, TestData, HP);
   // //PointCloudTool.visualizeORFApproachManualResult(OriginData, CurrentSemanticLabel, GroundResults);
   // //PointCloudTool.saveArrayFile("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Ground_ManualResult_7.txt", OriginData);

   // PosManualResultFilePathVector.clear();
   // NegManualResultFilePathVector.clear();
   // delete GroundModel;

   // //PointCloudTool.mapPreditcedLabel2OriginData(DLResultFilePath, OriginData);
   // PointCloudTool.saveArrayFile("E:\\PointCloudDataSet\\gt_Experiment\\conv_fea\\Approach.txt", OriginData);

   // return EXIT_SUCCESS;
}
