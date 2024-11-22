//
// Created by ateveraz on 19/11/24.
//

#include <iostream>
#include "AFNNC.h"
#include <Eigen/Dense>

AFNNC::AFNNC() {
    // Constructor
    //FourierNN *fourierNN = new FourierNN();
}

AFNNC::~AFNNC() {
    // Destructor
}

void AFNNC::reset() {
    ResetWeights();
    ResetGain();
}


void AFNNC::setTunningParameters(const float W0_, const float W1_, const float gamma0_, const float gamma1_, const float omega_, const float threshold_) {
    setFourierNN(W0_, W1_, omega_, threshold_);
    setAdaptiveIntegralGain(gamma0_, gamma1_);
}

Eigen::Vector3f AFNNC::computeAFNNC(Eigen::Vector3f& nu, Eigen::Vector3f& sigma, float t, float delta_t) {
    return computeFourierNN(nu, t, delta_t) + computeIntegralTerm(nu, sigma, delta_t);
}