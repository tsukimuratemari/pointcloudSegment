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

#include "online_rf.h"
#include <chrono>
#include <random>
#include <omp.h>
#include <mutex>


RandomTest::RandomTest(const int& numClasses, const int& numFeatures, const VectorXd &minFeatRange, const VectorXd &maxFeatRange) :
    m_numClasses(&numClasses), m_trueCount(0.0), m_falseCount(0.0),
    m_trueStats(VectorXd::Zero(numClasses)), m_falseStats(VectorXd::Zero(numClasses)) {
    m_feature = randDouble(0, numFeatures);
    m_threshold = randDouble(minFeatRange(m_feature), maxFeatRange(m_feature));
}

void RandomTest::update(const Sample& sample) {
    updateStats(sample, eval(sample));
}
    
bool RandomTest::eval(const Sample& sample) const {
    return (sample.x(m_feature) > m_threshold) ? true : false;
}
    
double RandomTest::score() const {
    double trueScore = 0.0, falseScore = 0.0, p;
    if (m_trueCount) {
        for (int nClass = 0; nClass < *m_numClasses; nClass++) {
            p = m_trueStats[nClass] / m_trueCount;
            trueScore += p * (1 - p);
        }
    }
        
    if (m_falseCount) {
        for (int nClass = 0; nClass < *m_numClasses; nClass++) {
            p = m_falseStats[nClass] / m_falseCount;
            falseScore += p * (1 - p);
        }
    }
        
    return (m_trueCount * trueScore + m_falseCount * falseScore) / (m_trueCount + m_falseCount + 1e-16);
}
    
pair<VectorXd, VectorXd > RandomTest::getStats() const {
    return pair<VectorXd, VectorXd> (m_trueStats, m_falseStats);
}

void RandomTest::updateStats(const Sample& sample, const bool& decision) {
    if (decision) {
        m_trueCount += sample.w;
        m_trueStats(sample.y) += sample.w;
    } else {
        m_falseCount += sample.w;
        m_falseStats(sample.y) += sample.w;
    }
}    

OnlineNode::OnlineNode(const Hyperparameters& hp, const int& numClasses, const int& numFeatures, const VectorXd& minFeatRange, const VectorXd& maxFeatRange, 
                       const int& depth) :
    m_numClasses(&numClasses), m_depth(depth), m_isLeaf(true), m_hp(&hp), m_label(-1),
    m_counter(0.0), m_parentCounter(0.0), m_labelStats(VectorXd::Zero(numClasses)),
    m_minFeatRange(&minFeatRange), m_maxFeatRange(&maxFeatRange) {
    // Creating random tests
    for (int nTest = 0; nTest < hp.numRandomTests; nTest++) {
        m_onlineTests.push_back(new RandomTest(numClasses, numFeatures, minFeatRange, maxFeatRange));
    }
}
    
OnlineNode::OnlineNode(const Hyperparameters& hp, const int& numClasses, const int& numFeatures, const VectorXd& minFeatRange, const VectorXd& maxFeatRange, 
                       const int& depth, const VectorXd& parentStats) :
    m_numClasses(&numClasses), m_depth(depth), m_isLeaf(true), m_hp(&hp), m_label(-1),
    m_counter(0.0), m_parentCounter(parentStats.sum()), m_labelStats(parentStats),
    m_minFeatRange(&minFeatRange), m_maxFeatRange(&maxFeatRange) {
    m_labelStats.maxCoeff(&m_label);
    // Creating random tests
    for (int nTest = 0; nTest < hp.numRandomTests; nTest++) {
        m_onlineTests.push_back(new RandomTest(numClasses, numFeatures, minFeatRange, maxFeatRange));
    }
}
    
OnlineNode::~OnlineNode() {
    if (!m_isLeaf) {
        delete m_leftChildNode;
        delete m_rightChildNode;
        delete m_bestTest;
    } else {
        for (int nTest = 0; nTest < m_hp->numRandomTests; nTest++) {
            delete m_onlineTests[nTest];
        }
    }
}
    
void OnlineNode::update(const Sample& sample) {
    m_counter += sample.w;
    m_labelStats(sample.y) += sample.w;

    if (m_isLeaf) {
        // Update online tests
        for (vector<RandomTest*>::iterator itr = m_onlineTests.begin(); itr != m_onlineTests.end(); ++itr) {
            (*itr)->update(sample);
        }

        // Update the label
        m_labelStats.maxCoeff(&m_label);

        // Decide for split
        if (shouldISplit()) {
            m_isLeaf = false;

            // Find the best online test
            int nTest = 0, minIndex = 0;
            double minScore = 1, score;
            for (vector<RandomTest*>::const_iterator itr(m_onlineTests.begin()); itr != m_onlineTests.end(); ++itr, nTest++) {
                score = (*itr)->score();
                if (score < minScore) {
                    minScore = score;
                    minIndex = nTest;
                }
            }
            m_bestTest = m_onlineTests[minIndex];
            for (int nTest = 0; nTest < m_hp->numRandomTests; nTest++) {
                if (minIndex != nTest) {
                    delete m_onlineTests[nTest];
                }
            }

            // Split
            pair<VectorXd, VectorXd> parentStats = m_bestTest->getStats();
            m_rightChildNode = new OnlineNode(*m_hp, *m_numClasses, m_minFeatRange->rows(), *m_minFeatRange, *m_maxFeatRange, m_depth + 1,
                                              parentStats.first);
            m_leftChildNode = new OnlineNode(*m_hp, *m_numClasses, m_minFeatRange->rows(), *m_minFeatRange, *m_maxFeatRange, m_depth + 1,
                                             parentStats.second);
        }
    } else {
        if (m_bestTest->eval(sample)) {
            m_rightChildNode->update(sample);
        } else {
            m_leftChildNode->update(sample);
        }
    }
}


void OnlineNode::eval(const Sample& sample, Result& result) {
    if (m_isLeaf) {
        if (m_counter + m_parentCounter) {
            result.confidence = m_labelStats / (m_counter + m_parentCounter);
            result.prediction = m_label;
        } else {
            result.confidence = VectorXd::Constant(m_labelStats.rows(), 1.0 / *m_numClasses);
            result.prediction = 0; 
        }
    } else {
        if (m_bestTest->eval(sample)) {
            m_rightChildNode->eval(sample, result);
        } else {
            m_leftChildNode->eval(sample, result);
        }
    }
}

bool OnlineNode::shouldISplit() const {
    bool isPure = false;
    for (int nClass = 0; nClass < *m_numClasses; nClass++) {
        if (m_labelStats(nClass) == m_counter + m_parentCounter) {
            isPure = true;
            break;
        }
    }

    if ((isPure) || (m_depth >= m_hp->maxDepth) || (m_counter < m_hp->counterThreshold)) {
        return false;
    } else {
        return true;
    }
}

OnlineTree::OnlineTree(const Hyperparameters& hp, const int& numClasses, const int& numFeatures, 
                       const VectorXd& minFeatRange, const VectorXd& maxFeatRange) :
    Classifier(hp, numClasses) {
    m_rootNode = new OnlineNode(hp, numClasses, numFeatures, minFeatRange, maxFeatRange, 0);
    m_name = "OnlineTree";
}

OnlineTree::~OnlineTree() {
    delete m_rootNode;
}
    
void OnlineTree::update(Sample& sample) {
    m_rootNode->update(sample);
}

void OnlineTree::eval(Sample& sample, Result& result) {
    m_rootNode->eval(sample, result);
}


OnlineRF::OnlineRF(const Hyperparameters& hp, const int& numClasses, const int& numFeatures, const VectorXd& minFeatRange, const VectorXd& maxFeatRange) :
    Classifier(hp, numClasses), m_counter(0.0), m_oobe(0.0) {
    OnlineTree *tree;
    for (int nTree = 0; nTree < hp.numTrees; nTree++) {
        tree = new OnlineTree(hp, numClasses, numFeatures, minFeatRange, maxFeatRange);
        m_trees.push_back(tree);
    }
    m_name = "OnlineRF";
}

OnlineRF::~OnlineRF() {
    for (int nTree = 0; nTree < m_hp->numTrees; nTree++) {
        delete m_trees[nTree];
    }
}
    
void OnlineRF::update(Sample& sample) {
    m_counter += sample.w;

    Result result(*m_numClasses), treeResult;

    //int numTries;
    //std::mutex Mutex;
    #pragma omp parallel for num_threads(16)
    for (int nTree = 0; nTree < m_hp->numTrees; nTree++) {
        int numTries = poisson(1.0);
        if (numTries) {
            for (int nTry = 0; nTry < numTries; nTry++) {
                m_trees[nTree]->update(sample);
            }
        } /*else {
            Mutex.lock();
            m_trees[nTree]->eval(sample, treeResult);
            result.confidence += treeResult.confidence;
            Mutex.unlock();
        }*/
    }

    int pre;
    result.confidence.maxCoeff(&pre);
    if (pre != sample.y) {
        m_oobe += sample.w;
    }
}

void OnlineRF::eval(Sample& sample, Result& result) {
    
    #pragma omp parallel for num_threads(20)
    for (int nTree = 0; nTree < m_hp->numTrees; nTree++) {
        Result treeResult;
        m_trees[nTree]->eval(sample, treeResult);
        result.confidence += treeResult.confidence;
    }

    result.confidence /= m_hp->numTrees;
    result.confidence.maxCoeff(&result.prediction);
}

void OnlineRF::_updateTreeByOOBE(DataSet& vDataSet)
{
	std::vector<pair<int,int>> OOBE4Trees(m_hp->numTrees);
    Result PredictResult;
	for (int i = 0; i < m_hp->numTrees; ++i)
	{
        OOBE4Trees[i]=std::make_pair(i,0);
		for (int k = 0; k < vDataSet.m_numSamples; ++k)
		{
			m_trees[i]->eval(vDataSet.m_samples[k], PredictResult);
			OOBE4Trees[i].second += PredictResult.prediction == vDataSet.m_samples[k].y ? 0 : 1;
		}
	}

    std::sort(OOBE4Trees.begin(), OOBE4Trees.end(), [](std::pair<int, int>& vLhs, std::pair<int, int>& vRhs) {return vLhs.second > vRhs.second; });

    int NumDiscardTree = 150;
    for (int i = 0; i < NumDiscardTree; i++)
    {
        delete m_trees[OOBE4Trees[i].first];
        m_trees[OOBE4Trees[i].first] = new OnlineTree(*m_hp,vDataSet.m_numClasses, vDataSet.m_numFeatures, vDataSet.m_minFeatRange, vDataSet.m_maxFeatRange);
    }
}

void OnlineRF::_updateTreeRandomly(const int vNumclass, const int vNumFeatures, const VectorXd& vMinFeatRange, const VectorXd& vMaxFeatRange)
{
    int NumDiscardTree = 75;
    for (int i = 0; i < NumDiscardTree; i++)
    {
        int RandomTreeIndex = _genRandomIntegerWithinRange(0, m_hp->numTrees - 1);
        delete m_trees[RandomTreeIndex];
        m_trees[RandomTreeIndex] = new OnlineTree(*m_hp, vNumclass, vNumFeatures, vMinFeatRange, vMaxFeatRange);
    }
}

int OnlineRF::_genRandomIntegerWithinRange(const int vMinRangeValue, const int vMaxRangeValue)
{
    std::default_random_engine Seed(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    std::uniform_int_distribution<> Gen(vMinRangeValue, vMaxRangeValue);
    return Gen(Seed);
}
