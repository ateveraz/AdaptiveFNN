//
// Created by ateveraz on 19/11/24.
//
#ifndef AFNNC_H
#define AFNNC_H

#include <iostream>
#include "FourierNN.h"
#include "AdapIntegralGain.h"



class AFNNC : public FourierNN, public AdapIntegralGain {
    public:
        AFNNC();
        ~AFNNC();

        void setTunningParameters(const float W0_, const float W1_, const float gamma0_, const float gamma1_, const float omega_, const float threshold_);
        Eigen::Vector3f computeAFNNC(Eigen::Vector3f& nu, Eigen::Vector3f& sigma, float t, float delta_t);
        void reset();

    /*
    private:
        FourierNN *fourierNN;
        AdapIntegralGain *adapIntegralGain;
     */
};

#endif // AFNNC_H