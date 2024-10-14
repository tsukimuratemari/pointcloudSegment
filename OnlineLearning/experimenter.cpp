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

#include <fstream>
#include <map>
#include "systime.h"
#include "experimenter.h"
#include "online_rf.h"
#include "online_mcboost.h"

void train(Classifier* model, DataSet& dataset, Hyperparameters& hp) {
    timeval startTime;
    gettimeofday(&startTime, NULL);

    if (model->name() == "OnlineRF")
    {
        OnlineRF* pRFModel = dynamic_cast<OnlineRF*>(model);
        //pRFModel->_updateTreeRandomly(dataset.m_numClasses, dataset.m_numFeatures, dataset.m_minFeatRange, dataset.m_maxFeatRange);
        //pRFModel->_updateTreeByOOBE(dataset);
    }
    if (model->name() == "OnlineMCBoost")
    {
        OnlineMCBoost* OMCBoost = dynamic_cast<OnlineMCBoost*>(model);
        OMCBoost->discardWeakLearner(dataset.m_numClasses, dataset.m_numFeatures, dataset.m_minFeatRange, dataset.m_maxFeatRange);
    }
    vector<int> randIndex;
    int sampRatio = dataset.m_numSamples / 10;
    vector<double> trainError(hp.numEpochs, 0.0);
    for (int nEpoch = 0; nEpoch < hp.numEpochs; nEpoch++) {
        //cout << "Epoch: " << nEpoch << endl;
        randPerm(dataset.m_numSamples, randIndex);
        for (int nSamp = 0; nSamp < dataset.m_numSamples; nSamp++) {
            if (hp.findTrainError) {
                Result result(dataset.m_numClasses);
                model->eval(dataset.m_samples[randIndex[nSamp]], result);
                if (result.prediction != dataset.m_samples[randIndex[nSamp]].y) {
                    trainError[nEpoch]++;
                }
            }

            model->update(dataset.m_samples[randIndex[nSamp]]);
            if (hp.verbose && (nSamp % sampRatio) == 0) {
                cout << "--- " << model->name() << " training --- Epoch: " << nEpoch + 1 << " --- ";
                //cout << (10 * nSamp) / sampRatio << "%";
                //cout << " --- Training error = " << trainError[nEpoch] << "/" << nSamp << endl;
            }
        }
    }

    timeval endTime;
    gettimeofday(&endTime, NULL);
    cout << "--- " << model->name() << " training time = ";
    cout << (endTime.tv_sec - startTime.tv_sec + (endTime.tv_usec - startTime.tv_usec) / 1e6) << " seconds." << endl;
}

vector<Result> test(Classifier* model, DataSet& dataset, Hyperparameters& hp) {
    timeval startTime;
    gettimeofday(&startTime, NULL);

    vector<Result> results;

    for (int nSamp = 0; nSamp < dataset.m_numSamples; nSamp++) {
        Result result(dataset.m_numClasses);
        model->eval(dataset.m_samples[nSamp], result);
        results.push_back(result);
    }

    double error = compError(results, dataset);
    if (hp.verbose) {
        cout << "--- " << model->name() << " test error: " << error << endl;
    }

    timeval endTime;
    gettimeofday(&endTime, NULL);
    cout << "--- " << model->name() << " testing time = ";
    cout << (endTime.tv_sec - startTime.tv_sec + (endTime.tv_usec - startTime.tv_usec) / 1e6) << " seconds." << endl;

    return results;
}

vector<Result> trainAndTest(Classifier* model, DataSet& dataset_tr, DataSet& dataset_ts, Hyperparameters& hp) {
    timeval startTime;
    gettimeofday(&startTime, NULL);

    if (model->name() == "OnlineRF")
    {
        OnlineRF* RFModel = dynamic_cast<OnlineRF*>(model);
        RFModel->_updateTreeRandomly(dataset_tr.m_numClasses, dataset_tr.m_numFeatures, dataset_tr.m_minFeatRange, dataset_tr.m_maxFeatRange);
    }
    vector<Result> results;
    vector<int> randIndex;
    int sampRatio = dataset_tr.m_numSamples / 10;
    vector<double> trainError(hp.numEpochs, 0.0);
    vector<double> testError;

    for (int nEpoch = 0; nEpoch < hp.numEpochs; nEpoch++) {
        randPerm(dataset_tr.m_numSamples, randIndex);
        for (int nSamp = 0; nSamp < dataset_tr.m_numSamples; nSamp++) {
            if (hp.findTrainError) {
                Result result(dataset_tr.m_numClasses);
                model->eval(dataset_tr.m_samples[randIndex[nSamp]], result);
                if (result.prediction != dataset_tr.m_samples[randIndex[nSamp]].y) {
                    trainError[nEpoch]++;
                }
            }

            model->update(dataset_tr.m_samples[randIndex[nSamp]]);
            if (hp.verbose && (nSamp % sampRatio) == 0) {
                cout << "--- " << model->name() << " training --- Epoch: " << nEpoch + 1 << " --- ";
                cout << (10 * nSamp) / sampRatio << "%";
                cout << " --- Training error = " << trainError[nEpoch] << "/" << nSamp << endl;
            }
        }

        results = test(model, dataset_ts, hp);
        testError.push_back(compError(results, dataset_ts));
    }

    timeval endTime;
    gettimeofday(&endTime, NULL);
    cout << "--- Total training and testing time = ";
    cout << (endTime.tv_sec - startTime.tv_sec + (endTime.tv_usec - startTime.tv_usec) / 1e6) << " seconds." << endl;

    if (hp.verbose) {
        cout << endl << "--- " << model->name() << " test error over epochs: ";
        dispErrors(testError);
    }

    // Write the results
    string saveFile = hp.savePath + ".errors";
    ofstream file(saveFile.c_str(), ios::binary);
    if (!file) {
        cout << "Could not access " << saveFile << endl;
        exit(EXIT_FAILURE);
    }
    file << hp.numEpochs << " 1" << endl;
    for (int nEpoch = 0; nEpoch < hp.numEpochs; nEpoch++) {
        file << testError[nEpoch] << endl;
    }
    file.close();

    return results;
}

double compError(const vector<Result>& results, const DataSet& dataset) {
    double error = 0.0;
    for (int nSamp = 0; nSamp < dataset.m_numSamples; nSamp++) {
        if (results[nSamp].prediction != dataset.m_samples[nSamp].y) {
            error++;
        }
    }

    return error / dataset.m_numSamples;
}

void dispErrors(const vector<double>& errors) {
    for (int nSamp = 0; nSamp < (int) errors.size(); nSamp++) {
        cout << nSamp + 1 << ": " << errors[nSamp] << " --- ";
    }
    cout << endl;
}

vector<Eigen::VectorXd> concatSVAndResult(const std::string& vSVInfoFile, const vector<Result>& vResult)
{
    //load SVInfo
    std::ifstream Reader(vSVInfoFile);
    std::string Line;
    const int LineLength = 27;
    const int SVInfoLength = 8;
    const int ResultLength = 2;
    vector<Eigen::VectorXd> SVAndResult;
    while (std::getline(Reader, Line))
    {
		Eigen::VectorXd SV((SVInfoLength + ResultLength));
        std::stringstream SS(Line);
        //load coordinate and color
        for (int i = 0; i < 6; i++)
        {
            std::string SVInfo;
            std::getline(SS, SVInfo, ' ');
            SV[i] = std::stod(SVInfo);
        }
        //load SV serial number
        const int SVSerialNumberIndex = 25;
        const int SVLabelIndex = 26;
        for (int i = 6; i < LineLength; i++)
        {
            std::string SVInfo;
            std::getline(SS, SVInfo, ' ');
            if (i == SVSerialNumberIndex)
                SV[6] = std::stod(SVInfo);
            if (i == SVLabelIndex)
                SV[7] = std::stod(SVInfo);  
        }
        SVAndResult.emplace_back(SV);
    }
    //load Result
    _ASSERT(SVAndResult.size() == vResult.size());
    for (int i = 0; i < vResult.size(); i++)
    {
        SVAndResult[i][SVInfoLength] = vResult[i].confidence.maxCoeff();
        SVAndResult[i][SVInfoLength + 1] = vResult[i].prediction;
    }
    Reader.close();
    return SVAndResult;
}

//deal conflict one by one
void dealConfilict(vector<Eigen::VectorXd>& vPrectionResult,
    const std::uint32_t vSemanticLabel, std::map<std::uint32_t, Eigen::VectorXd>& voFinalResult)
{
    for (auto& SVResult : vPrectionResult)
    {
        //only deal positive predction
        float minConfidenceThreshold = 0;
        if (static_cast<std::uint32_t>(SVResult[9])&&SVResult[8]> minConfidenceThreshold)
        {
            SVResult[9] = vSemanticLabel;
            if (voFinalResult.find(SVResult[6]) != voFinalResult.end())
            {
                //pick the predction of max confidence as the final result; 
                if (SVResult[8] > voFinalResult.at(SVResult[6])[8])
                {
                    voFinalResult.at(SVResult[6])[8] = SVResult[8];
                    voFinalResult.at(SVResult[6])[9] = SVResult[9];
                }
            }
            else
                voFinalResult.insert(std::make_pair(static_cast<std::uint32_t>(SVResult[6]), SVResult));
        }
    }
}

vector<Eigen::VectorXd> mapResult2OriginSVPointCloud(const std::string& vOriginSVPointCloud,
    const std::map<std::uint32_t, Eigen::VectorXd>& vResult, const bool vIsSaveFeatures)
{
    vector<Eigen::VectorXd> SVs;
    loadSVPointCloud(vOriginSVPointCloud, SVs, vIsSaveFeatures);
    if (vIsSaveFeatures)
    {
        for (auto& SV : SVs)
        {
            if (vResult.find(SV[25]) != vResult.end())
                SV[26] = vResult.at(SV[25])[9];
        }
    }
    else
    {
        for (auto& SV : SVs)
        {
            if (vResult.find(SV[6]) != vResult.end())
                SV[7] = vResult.at(SV[6])[9];
        }
    }
    return SVs;
}

void loadSVPointCloud(const std::string& vOriginSVPointCloud, vector<Eigen::VectorXd>& voSVs,const bool vIsSaveFeatures)
{
    std::ifstream Reader(vOriginSVPointCloud);
    std::string Line;
    const int LineLength = 27;
    if (vIsSaveFeatures)
    {
        const int NeededItemNum = 27;
        while (std::getline(Reader, Line))
        {
            Eigen::VectorXd SV(NeededItemNum);
            std::stringstream SS(Line);
            std::string SVInfo;
            std::uint32_t Count = 0;
            while (std::getline(SS, SVInfo, ' '))
            {
                SV[Count++] = std::stod(SVInfo);
            }
            voSVs.emplace_back(SV);
        }
    }
    else
    {
        const int NeededItemNum = 8;
        while (std::getline(Reader, Line))
        {
            Eigen::VectorXd SV(NeededItemNum);
            std::stringstream SS(Line);
            //load coordinate and color
            for (int i = 0; i < 6; i++)
            {
                std::string SVInfo;
                std::getline(SS, SVInfo, ' ');
                SV[i] = std::stod(SVInfo);
            }
            //load SV serial number
            const int SVSerialNumberIndex = 25;
            const int SVLabelIndex = 26;
            for (int i = 6; i < LineLength; i++)
            {
                std::string SVInfo;
                std::getline(SS, SVInfo, ' ');
                if (i == SVSerialNumberIndex)
                    SV[6] = std::stod(SVInfo);
                if (i == SVLabelIndex)
                    SV[7] = std::stod(SVInfo);
            }
            voSVs.emplace_back(SV);
        }
    }
}

void saveSVAndResult(const vector<Eigen::VectorXd>& vSVAndResult, const std::string& vSavePath)
{
    std::ofstream Writer(vSavePath);
    for (auto& SV : vSVAndResult)
    {
        for (int i = 0; i < SV.size(); i++)
        {
            Writer << SV[i];
            if (i != SV.size() - 1)
                Writer << " ";
            else
                Writer << "\n";
        }
    }
    Writer.close();
}
