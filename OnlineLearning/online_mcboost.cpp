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

#include "online_mcboost.h"
#include "online_rf.h"
#include <chrono>
#include <random>

OnlineMCBoost::OnlineMCBoost(const Hyperparameters& hp, const int& numClasses, const int& numFeatures, 
                             const VectorXd& minFeatRange, const VectorXd& maxFeatRange) :
    Booster(hp, numClasses, numFeatures, minFeatRange, maxFeatRange) {
    m_name = "OnlineMCBoost";
}


void OnlineMCBoost::update(Sample& sample) {
    sample.w = 1.0;
    double fy = 0.0;
    for (int nBase = 0; nBase < m_hp->numBases; nBase++) {
        
        Result baseResult(*m_numClasses);
        m_bases[nBase]->update(sample);
        //m_bases[nBase]->eval(sample, baseResult);
        fy += m_hp->shrinkage * (baseResult.confidence(sample.y) - 1.0 / (*m_numClasses));

        switch (m_hp->lossFunction) {
        case EXPONENTIAL: {
            sample.w = d_exp(fy);
            break;
        }
        case LOGIT: {
            sample.w = d_logit(fy);
            break;
        }
        }
    }
}

void OnlineMCBoost::discardWeakLearner(const int vNumclass, const int vNumFeatures, const VectorXd& vMinFeatRange, const VectorXd& vMaxFeatRange)
{
    for (int i = 0; i < m_hp->numBases; i++)
    {
        OnlineRF* ORF = dynamic_cast<OnlineRF*>(m_bases[i]);
        ORF->_updateTreeRandomly(vNumclass, vNumFeatures, vMinFeatRange, vMaxFeatRange);

        int NumDiscardTree = 75;
        for (int i = 0; i < NumDiscardTree; i++)
        {
            int RandomWeakLearnerIndex = _genRandomIntegerWithinRange(0, m_hp->numTrees - 1);
            delete m_bases[RandomWeakLearnerIndex];
            m_bases[RandomWeakLearnerIndex] = new OnlineRF(*m_hp, vNumclass, vNumFeatures, vMinFeatRange, vMaxFeatRange);
        }
    }
}

int OnlineMCBoost::_genRandomIntegerWithinRange(const int vMinRangeValue, const int vMaxRangeValue)
{
    std::default_random_engine Seed(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    std::uniform_int_distribution<> Gen(vMinRangeValue, vMaxRangeValue);
    return Gen(Seed);
}
