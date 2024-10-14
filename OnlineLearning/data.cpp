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

#include "data.h"
#include "Utils.h"
DataSet::DataSet()
{

}

DataSet::DataSet(vector<vector<double>>& vTrainData)
{
    m_numFeatures=9;
    uint32_t LabelIndex = 12;
    m_FirstFeatureColumnIndex = 3;
    m_numSamples = vTrainData.size();
    m_samples.clear();
    set<int> labels;
    for (int i = 0; i < m_numSamples; i++) 
    {
        Sample sample;
        sample.x = VectorXd(m_numFeatures);
        sample.id = i;
        sample.w = 1.0;
        sample.y = static_cast<int>(vTrainData[i][LabelIndex]);
        labels.insert(sample.y);
        for (int k = 0; k < m_numFeatures; k++)
        {   
            sample.x(k) = vTrainData[i][k + m_FirstFeatureColumnIndex];
        }
       
        m_samples.push_back(sample); 
        // push sample into dataset
    }
    m_numClasses = labels.size();

    // Find the data range
    findFeatRange();

    cout << "Loaded " << m_numSamples << " samples with " << m_numFeatures;
    cout << " features and " << m_numClasses << " classes." << endl;
}

void DataSet::updateLabel(vector<Result>& viResult)
{
    if (m_numSamples != viResult.size()) 
    {
        cout << "result size isn't matching with dataset" << endl;
        return;
    }
    for (int i = 0; i < m_samples.size(); i++)
    {
        m_samples[i].y = viResult[i].prediction;
    }
}

void DataSet::updateConf(vector<Result>& viResult)
{
	if (m_numSamples != viResult.size())
	{
		cout << "result size isn't matching with dataset" << endl;
		return;
	}
	for (int i = 0; i < m_samples.size(); i++)
	{
        float Confidence = viResult[i].confidence.maxCoeff();
        if (Confidence > m_samples[i].confidence)
        {
            m_samples[i].confidence = Confidence;
            m_samples[i].y = viResult[i].prediction;
        }
	}
}

void DataSet::loadApproachingData(string& vFilePath)
{
	ifstream Reader(vFilePath);
	string FileLine;
	int i = 0;
	set<int> labels;
    m_samples.clear();
	while (getline(Reader, FileLine))
	{
		stringstream StringStream(FileLine);
        VectorXd CoordRGB(m_FirstFeatureColumnIndex);
        for (int k = 0; k < m_FirstFeatureColumnIndex; k++)
            StringStream >> CoordRGB(k);
        m_CoordRGB.push_back(CoordRGB);
        Sample sample;
		sample.x = VectorXd(m_numFeatures);
		sample.id = i;
		sample.w = 1.0;
		for (int k = 0; k < m_numFeatures; k++)
		{
			StringStream >> sample.x(k);
		}
		StringStream >> sample.y;
		if (sample.y == m_SemanticCategory)
			sample.y = static_cast<int>(EBiClassificationSemanticCategory::PositiveCategory);
		else
			sample.y = static_cast<int>(EBiClassificationSemanticCategory::NegativeCategory);
		labels.insert(sample.y);
		m_samples.push_back(sample);
		i++;
	}
	m_numSamples = m_samples.size();
	m_numClasses = labels.size();
	findFeatRange();
	Reader.close();
}

void DataSet::writeResult(string& voFilePath)
{
	ofstream Writer(voFilePath);
	Writer.setf(ios::fixed, ios::floatfield);
	Writer.precision(3);
    for (int i = 0; i < m_samples.size(); i++)
    {
        for (int k = 0; k < m_FirstFeatureColumnIndex; k++)
        {
            Writer << m_CoordRGB[i](k) << " ";
        }
        for (int k = 0; k < m_numFeatures; k++)
        {
            Writer << m_samples[i].x(k) << " ";
        }
        Writer << m_samples[i].y << endl;
    }
	Writer.close();
}

void DataSet::loadManualData(string& vPositiveFilePath, string& vNegativeFilePath)
{
	ifstream Reader(vPositiveFilePath);
	string FileLine;
	int i = 0;
	set<int> labels;
	while (getline(Reader, FileLine))
	{
		stringstream StringStream(FileLine);
		VectorXd CoordRGB = VectorXd(m_FirstFeatureColumnIndex);
		for (int k = 0; k < m_FirstFeatureColumnIndex; k++)
			StringStream >> CoordRGB(k);
		m_CoordRGB.push_back(CoordRGB);
		Sample sample;
		sample.x = VectorXd(m_numFeatures);
		sample.id = i;
		sample.w = 1.0;
		for (int k = m_FirstFeatureColumnIndex; k < m_FirstFeatureColumnIndex + m_numFeatures; k++)
		{
			StringStream >> sample.x(k);
		}
		StringStream >> sample.y;
		sample.y = static_cast<int>(EBiClassificationSemanticCategory::PositiveCategory);
		labels.insert(sample.y);
		m_samples.push_back(sample);
		i++;
	}
    Reader.close();
	ifstream NegReader(vNegativeFilePath);
	while (getline(NegReader, FileLine))
	{
		stringstream StringStream(FileLine);
		VectorXd CoordRGB = VectorXd(m_FirstFeatureColumnIndex);
		for (int k = 0; k < m_FirstFeatureColumnIndex; k++)
			StringStream >> CoordRGB(k);
		m_CoordRGB.push_back(CoordRGB);
		Sample sample;
		sample.x = VectorXd(m_numFeatures);
		sample.id = i;
		sample.w = 1.0;
		for (int k = m_FirstFeatureColumnIndex; k < m_FirstFeatureColumnIndex + m_numFeatures; k++)
		{
			StringStream >> sample.x(k);
		}
		StringStream >> sample.y;
		sample.y = static_cast<int>(EBiClassificationSemanticCategory::NegativeCategory);
		labels.insert(sample.y);
		m_samples.push_back(sample);
		i++;
	}
	m_numSamples = m_samples.size();
	m_numClasses = labels.size();
	findFeatRange();
	NegReader.close();
}



void CBuildingDataSet::setColumnIndex()
{
	m_numFeatures = 2;
	m_FirstFeatureColumnIndex = 6;
	m_LabelIndex = 8;
	m_SemanticCategory = static_cast<int>(ESemanticCategory::Building);
}


void DataSet::findFeatRange() {

    m_minFeatRange = VectorXd(m_numFeatures);
    m_maxFeatRange = VectorXd(m_numFeatures);
    double minVal, maxVal;
    for (int nFeat = 0; nFeat < m_numFeatures; nFeat++) {
        minVal = m_samples[0].x(nFeat);
        maxVal = m_samples[0].x(nFeat);
        for (int nSamp = 1; nSamp < m_numSamples; nSamp++) {
            if (m_samples[nSamp].x(nFeat) < minVal) {
                minVal = m_samples[nSamp].x(nFeat);
            }
            if (m_samples[nSamp].x(nFeat) > maxVal) {
                maxVal = m_samples[nSamp].x(nFeat);
            }
        }
        m_minFeatRange(nFeat) = minVal;
        m_maxFeatRange(nFeat) = maxVal;
    }
}

void DataSet::load(const string& x_filename, const string& y_filename) {
    ifstream xfp(x_filename.c_str(), ios::binary);
    if (!xfp) {
        cout << "Could not open input file " << x_filename << endl;
        exit(EXIT_FAILURE);
    }
    ifstream yfp(y_filename.c_str(), ios::binary);
    if (!yfp) {
        cout << "Could not open input file " << y_filename << endl;
        exit(EXIT_FAILURE);
    }
    cout << "Loading data file: " << x_filename << " ... " << endl;

    // Reading the header
    int tmp;
    xfp >> m_numSamples;
    xfp >> m_numFeatures;
    yfp >> tmp;
    if (tmp != m_numSamples) {
        cout << "Number of samples in data and labels file is different" << endl;
        exit(EXIT_FAILURE);
    }
    yfp >> tmp;

    m_samples.clear();
    set<int> labels;
    for (int nSamp = 0; nSamp < m_numSamples; nSamp++) {
        Sample sample;
        sample.x = VectorXd(m_numFeatures);
        sample.id = nSamp;
        sample.w = 1.0;
        yfp >> sample.y;
        labels.insert(sample.y);
        for (int nFeat = 0; nFeat < m_numFeatures; nFeat++) {
            xfp >> sample.x(nFeat);
        }
        m_samples.push_back(sample); // push sample into dataset
    }
    xfp.close();
    yfp.close();
    m_numClasses = labels.size();

    // Find the data range
    findFeatRange();

    cout << "Loaded " << m_numSamples << " samples with " << m_numFeatures;
    cout << " features and " << m_numClasses << " classes." << endl;
}

void DataSet::load(vector<vector<double>>& vTrainData)
{
    _ASSERTE(m_LabelIndex != m_EmptyIndex);
    m_numSamples = vTrainData.size();
    m_samples.clear();
    set<int> labels;
    for (int i = 0; i < m_numSamples; i++)
    {
        Sample sample;
        sample.x = VectorXd(m_numFeatures);
        sample.id = i;
        sample.w = 1.0;
        sample.y = static_cast<int>(vTrainData[i][m_LabelIndex]);
        labels.insert(sample.y);
        for (int k = 0; k < m_numFeatures; k++)
        {
            sample.x(k) = vTrainData[i][k + m_FirstFeatureColumnIndex];
        }

        m_samples.push_back(sample);
        // push sample into dataset
    }
    m_numClasses = labels.size();

    // Find the data range
    findFeatRange();

    cout << "Loaded " << m_numSamples << " samples with " << m_numFeatures;
    cout << " features and " << m_numClasses << " classes." << endl;
}

Result::Result() 
{

}

Result::Result(const int& numClasses) : confidence(VectorXd::Zero(numClasses)) {
}

Cache::Cache() : margin(-1.0), yPrime(-1) {
}

Cache::Cache(const Sample& sample, const int& numBases, const int& numClasses) : margin(-1.0), yPrime(-1) {
    cacheSample.x = sample.x;
    cacheSample.y = sample.y;
    cacheSample.w = sample.w;
    cacheSample.id = sample.id;
}

