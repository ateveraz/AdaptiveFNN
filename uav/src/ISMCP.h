//
// Created by ateveraz on 23/11/24.
//

#ifndef ISMCP_H
#define ISMCP_H

#include <iostream>
#include <Eigen/Dense>
#include "NMethods.h"


class ISMCP {
    public:
        ISMCP();
        ~ISMCP();

        void resetISMCP();
        void setISMCP(const float m_, const float g_, Eigen::Matrix3f alphap_, Eigen::Matrix3f gammap_, Eigen::Matrix3f Kpm_);
        void solveSubactuation(const float delta_t, Eigen::Quaternionf q, Eigen::Vector3f xie, Eigen::Vector3f xiep, Eigen::Vector3f xid, Eigen::Vector3f xidpp, Eigen::Vector3f xidppp, Eigen::Vector3f ez);
        Eigen::Vector3f getWd();
        Eigen::Quaternionf getQd();

    private:
        float sech(float value);


        float delta_t;

        Eigen::Vector3f sgnpos_p, sgnpos, sgnori_p, sgnori;
        Eigen::Matrix3f I = Eigen::Matrix3f::Identity(3,3);

        Eigen::Vector3f wd;
        Eigen::Quaternionf qd;

        Eigen::Matrix3f alphap, gammap, Kpm;

        float m, g;

};



#endif //ISMCP_H
