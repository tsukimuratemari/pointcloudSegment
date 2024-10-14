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

#ifndef DATA_H_
#define DATA_H_

#include <fstream>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <set>
#include <string>
#include <Eigen/Core>
#include <Eigen/src/Core/Array.h>

using namespace std;
using namespace Eigen;



// DATA CLASSES
class Sample {
public:
    VectorXd x;
    int y;
    double w;
    int id;
    double confidence=0.0f;
};

class Result {
public:
	Result();
	Result(const int& numClasses);

	VectorXd confidence;
	int prediction;
};

class DataSet {
 public:
    DataSet();
    DataSet(vector<vector<double>>& vTrainData);
    DataSet(int vNumFeatures, int vFirstFeatureColumnIndex, int vLabelIndex) :m_numFeatures(vNumFeatures), m_FirstFeatureColumnIndex(vFirstFeatureColumnIndex), m_LabelIndex(vLabelIndex) {};

    virtual void setColumnIndex() {};

    void writeResult(string& voFilePath);
    void updateConf(vector<Result>& viResult);
    void updateLabel(vector<Result>& viResult);
    void loadApproachingData(string& vFilePath);
    void loadManualData(string& vPositiveFilePath, string& vNegativeFilePath);
    void findFeatRange();

    void load(const string& x_filename, const string& y_filename);
    void load(vector<vector<double>>& vTrainData);
    void setNumFeatures(int vNumFeatures) { m_numFeatures = vNumFeatures; }
    void setLaeblIndex(int vLabelIndex) { m_LabelIndex = vLabelIndex; }
    void setFirstFeatureColumnIndex(int vFirstFeatureColumnIndex) { m_FirstFeatureColumnIndex = vFirstFeatureColumnIndex; }

    vector<VectorXd> m_CoordRGB;
    vector<Sample> m_samples;
	int m_numSamples = 0;
	int m_numFeatures = 19;
	int m_numClasses = 0;
	int m_FirstFeatureColumnIndex = 6;
	int m_LabelIndex = -1;
	int m_EmptyIndex = -1;
    int m_SemanticCategory;
    VectorXd m_minFeatRange;
    VectorXd m_maxFeatRange;
};

class CBuildingDataSet :public DataSet
{
    void setColumnIndex()override;
};

class CTreeDataSet :public DataSet
{
    void setColumnIndex()override;
};

class CVehicleDataSet :public DataSet
{
    void setColumnIndex()override;
};

class CGroundDataSet :public DataSet
{
    void setColumnIndex()override;
};



class Cache {
public:
    Cache();
    Cache(const Sample& sample, const int& numBases, const int& numClasses);

    Sample cacheSample;
    double margin;
    int yPrime; // Class with closest margin to the sample
};
#endif /* DATA_H_ */
