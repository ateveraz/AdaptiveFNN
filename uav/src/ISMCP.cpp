//
// Created by ateveraz on 23/11/24.
//

#include "ISMCP.h"
#include "NMethods.h"
#include <Eigen/Dense>

ISMCP::ISMCP() {

    resetISMCP();
}

ISMCP::~ISMCP() {}

void ISMCP::resetISMCP() {
    sgnpos_p = Eigen::Vector3f::Zero();
    sgnpos = Eigen::Vector3f::Zero();
    sgnori_p = Eigen::Vector3f::Zero();
    sgnori = Eigen::Vector3f::Zero();
}

void ISMCP::setISMCP(const float m_, const float g_, Eigen::Matrix3f alphap_, Eigen::Matrix3f gammap_, Eigen::Matrix3f Kpm_) {
    m = m_;
    g = g_;
    alphap = alphap_;
    gammap = gammap_;
    Kpm = Kpm_;
}

void ISMCP::solveSubactuation(const float delta_t, Eigen::Quaternionf q, Eigen::Vector3f xie, Eigen::Vector3f xiep, Eigen::Vector3f xid, Eigen::Vector3f xidpp, Eigen::Vector3f xidppp) {

    Eigen::Vector3f nup = xiep + alphap*xie;

    sgnpos_p = signth(nup,1);
    sgnpos = rk4_vec(sgnpos, sgnpos_p, delta_t);

    Eigen::Vector3f nurp = nup + gammap*sgnpos;

    Eigen::Vector3f xirpp = xidpp - alphap*xiep - gammap*sgnpos_p;

    Eigen::Vector3f u = -Kpm*nurp - m*g*ez + m*xirpp;

    Trs = u.norm();

    Eigen::Vector3f Qe3 = q.toRotationMatrix()*ez;

    Eigen::Vector3f Lambpv(powf(sech(nup(0)*1),2), powf(sech(nup(1)*1),2), powf(sech(nup(2)*1),2) );
    Eigen::Matrix3f Lambp = Lambpv.asDiagonal();

    Eigen::Vector3f up;

    up = -(Kpm + m*alphap + m*gammap*Lambp) * (g*ez - (Trs/m)*Qe3 - xidpp)
         -alphap*(Kpm + m*gammap*Lambp)*xiep - Kpm*gammap*sgnpos_p + m*xidppp;

    Eigen::Vector3f uh = u.normalized();
    Eigen::Vector3f uph = ((u.transpose()*u)*up - (u.transpose()*up)*u)/(powf(u.norm(),3));

    Eigen::Quaternionf qd2( (0.5)*sqrtf(-2*uh(2)+2) , uh(1)/sqrtf(-2*uh(2)+2), -uh(0)/sqrtf(-2*uh(2)+2), 0);

    Eigen::Vector3f wd2(uph(1) - ( (uh(1)*uph(2))/(1-uh(2)) ),
                        -uph(0) + ( (uh(0)*uph(2))/(1-uh(2)) ),
                        (uh(1)*uph(0) - uh(0)*uph(1))/(1-uh(2)));

    wd = wd2;
    qd = qd2;
}

Eigen::Vector3f ISMCP::getWd() {
    return wd;
}

Eigen::Quaternionf ISMCP::getQd() {
    return qd;
}

float ISMCP::getTrs() {
    return Trs;
}

float ISMCP::sech(float value) {
    return 1 / coshf(value);
}

