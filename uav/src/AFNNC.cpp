//
// Created by ateveraz on 19/11/24.
//

#include <iostream>
#include "AFNNC.h"
#include <Eigen/Dense>
#include <Matrix.h>
#include <GroupBox.h>
#include <DoubleSpinBox.h>
#include <LayoutPosition.h>
#include <FrameworkManager.h>
#include <Matrix.h>
#include <MatrixDescriptor.h>
#include <Tab.h>

using std::string;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::filter;

namespace flair
{
namespace filter
{
AFNNC::AFNNC(const LayoutPosition *position, string name) : ControlLaw(position->getLayout(), name, 4), FourierNN(), AdapIntegralGain(), ISMCP(){
    previous_chrono_time = std::chrono::high_resolution_clock::now();
    firstUpdate = true;

	// Mutex for data access.
    input = new Matrix(this, 4, 8, floatType, name);

    // Label descriptor for data saved.
    MatrixDescriptor* desc = new MatrixDescriptor(23,1);
    desc->SetElementName(0,0,"q0");
    desc->SetElementName(1,0,"q1");
    desc->SetElementName(2,0,"q2");
    desc->SetElementName(3,0,"q3");
    desc->SetElementName(4,0,"wx");
    desc->SetElementName(5,0,"wy");
    desc->SetElementName(6,0,"wz");
    desc->SetElementName(7,0,"px");
    desc->SetElementName(8,0,"py");
    desc->SetElementName(9,0,"pz");
    desc->SetElementName(10,0,"u_roll");
    desc->SetElementName(11,0,"u_pitch");
    desc->SetElementName(12,0,"u_yaw");
    desc->SetElementName(13,0,"thrust");
    desc->SetElementName(14,0,"ecx");
    desc->SetElementName(15,0,"ecy");
    desc->SetElementName(16,0,"ecz");
    desc->SetElementName(17,0,"udeTx");
    desc->SetElementName(18,0,"udeTy");
    desc->SetElementName(19,0,"udeTz");
    desc->SetElementName(20,0,"udeRx");
    desc->SetElementName(21,0,"udeRy");
    desc->SetElementName(22,0,"udeRz");
    exp_data = new Matrix(this,desc,floatType,name);
    delete desc;

    /*
        * GroupBox for user-defined parameters
     */
    GroupBox *reglages_groupbox = new GroupBox(position, name);
    GroupBox *pos = new GroupBox(reglages_groupbox->NewRow(), "SMC Position");
    GroupBox *smc = new GroupBox(reglages_groupbox->NewRow(), "SMC Attitude");
    GroupBox *fourier = new GroupBox(reglages_groupbox->NewRow(), "FNN controller");
    GroupBox *integral_term = new GroupBox(reglages_groupbox->NewRow(), "Integral adaptive gain");
    GroupBox *mot = new GroupBox(reglages_groupbox->NewRow(), "Motor characterization");

    // User-defined for trust PD controller
    gamma_x = new DoubleSpinBox(pos->NewRow(), "gamma_x:", 0, 500, 0.001, 3);
    gamma_y = new DoubleSpinBox(pos->LastRowLastCol(), "gamma_y:", 0, 500, 0.001, 3);
    gamma_z = new DoubleSpinBox(pos->LastRowLastCol(), "gamma_z:", 0, 500, 0.001, 3);
    alpha_x = new DoubleSpinBox(pos->NewRow(), "alpha_x:", 0, 50000, 0.5, 3);
    alpha_y = new DoubleSpinBox(pos->LastRowLastCol(), "alpha_y:", 0, 50000, 0.5, 3);
    alpha_z = new DoubleSpinBox(pos->LastRowLastCol(), "alpha_z:", 0, 50000, 0.5, 3);
    Kp_x = new DoubleSpinBox(pos->NewRow(), "Kp_x:", 0, 50000, 0.5, 3);
    Kp_y = new DoubleSpinBox(pos->LastRowLastCol(), "Kp_y:", 0, 50000, 0.5, 3);
    Kp_z = new DoubleSpinBox(pos->LastRowLastCol(), "Kp_z:", 0, 50000, 0.5, 3);
    m = new DoubleSpinBox(pos->NewRow(),"m",0,2000,0.001,3);
    g = new DoubleSpinBox(pos->LastRowLastCol(),"g",-10,10,0.01,3);
    T = new DoubleSpinBox(pos->NewRow(), "period, 0 for auto", " s", 0, 1, 0.01,3);

    // User-defined for attitude SMC
    alpha_roll = new DoubleSpinBox(smc->NewRow(), "Alpha_roll:", 0, 50000, 0.0001, 4);
    alpha_pitch = new DoubleSpinBox(smc->LastRowLastCol(), "Alpha_pitch:", 0, 50000, 0.0001, 4);
    alpha_yaw = new DoubleSpinBox(smc->LastRowLastCol(), "Alpha_yaw:", 0, 50000, 0.0001, 4);
    Kd_roll = new DoubleSpinBox(smc->NewRow(), "Kd_roll:", 0, 50000, 0.0001, 4);
    Kd_pitch = new DoubleSpinBox(smc->LastRowLastCol(), "Kd_pitch:", 0, 50000, 0.0001, 4);
    Kd_yaw = new DoubleSpinBox(smc->LastRowLastCol(), "Kd_yaw:", 0, 50000, 0.0001, 4);
    k = new DoubleSpinBox(smc->NewRow(), "k:", 0, 50000, 0.05, 2);
    p = new DoubleSpinBox(smc->LastRowLastCol(), "p:", 0, 50000, 1, 1);

    // User-defined for FNN controller
    W0_roll = new DoubleSpinBox(fourier->NewRow(), "W0_roll:", 0, 500, 0.00001, 5);
    W0_pitch = new DoubleSpinBox(fourier->LastRowLastCol(), "W0_pitch:", 0, 500, 0.00001, 5);
    W0_yaw = new DoubleSpinBox(fourier->LastRowLastCol(), "W0_yaw:", 0, 500, 0.00001, 5);
    freq = new DoubleSpinBox(fourier->NewRow(), "Fundamental frequency:", 0, 50000, 0.0001, 5);
    W1_fnn = new DoubleSpinBox(fourier->LastRowLastCol(), "Security feedback gain (W1):", 0, 50000, 0.0001, 5);
    threshold = new DoubleSpinBox(fourier->LastRowLastCol(), "Threshold (eps) for monitor:", 0, 10, 0.0001, 5);

    // User-defined for integral adaptive term
    Gamma0_roll = new DoubleSpinBox(integral_term->NewRow(), "Lambda_roll:", 0, 50000, 0.00001, 5);
    Gamma0_pitch = new DoubleSpinBox(integral_term->LastRowLastCol(), "Lambda_pitch:", 0, 50000, 0.00001, 5);
    Gamma0_yaw = new DoubleSpinBox(integral_term->LastRowLastCol(), "Lambda_yaw:", 0, 50000, 0.00001, 5);
    Gamma1_int = new DoubleSpinBox(integral_term->NewRow(), "Security feedback gain (Gamma1):", 0, 50000, 0.00001, 5);

    // User-defined for motor characterization
    sat_r = new DoubleSpinBox(mot->NewRow(), "sat roll:", 0, 1, 0.1);
    sat_p = new DoubleSpinBox(mot->LastRowLastCol(), "sat pitch:", 0, 1, 0.1);
    sat_y = new DoubleSpinBox(mot->LastRowLastCol(), "sat yaw:", 0, 1, 0.1);
    sat_t = new DoubleSpinBox(mot->NewRow(), "sat thrust:", 0, 1, 0.1);
    km = new DoubleSpinBox(mot->LastRowLastCol(), "km:", -10, 10, 0.01, 3);

    t0 = double(GetTime())/1000000000;

    reset();

    AddDataToLog(exp_data);
}

AFNNC::~AFNNC() {
    // Destructor
}

void AFNNC::reset() {
    ResetWeights();
    ResetGain();
    resetISMCP();
}

void AFNNC::cartesian2configuration(Eigen::Quaternionf &qd, Eigen::Vector3f &wfd, const float delta_t, Eigen::Quaternionf q, Eigen::Vector3f xie, Eigen::Vector3f xiep, Eigen::Vector3f xid, Eigen::Vector3f xidpp, Eigen::Vector3f xidppp, Eigen::Vector3f ez)
{
    solveSubactuation(delta_t, q, xie, xiep, xid, xidpp, xidppp, ez);
    qd = getQd();
    wfd = getWd();
}

void AFNNC::setISMC(float m_, float g_, Eigen::Matrix3f alphap_, Eigen::Matrix3f gammap_, Eigen::Matrix3f Kpm_) {
    setISMCP(m_, g_, alphap_, gammap_, Kpm_);
}

void AFNNC::setCustomController() {
 	Eigen::Vector3f W0v_((float)W0_roll->Value(), (float)W0_pitch->Value(), (float)W0_yaw->Value());
    Eigen::Matrix3f W0_ = W0v_.asDiagonal();
    Eigen::Vector3f Gamma0v_((float)Gamma0_roll->Value(), (float)Gamma0_pitch->Value(), (float)Gamma0_yaw->Value());
    Eigen::Matrix3f Gamma0_ = Gamma0v_.asDiagonal();
    Eigen::Vector3f GammaPv_((float)gamma_x->Value(), (float)gamma_y->Value(), (float)gamma_z->Value());
    Eigen::Matrix3f GammaP_ = GammaPv_.asDiagonal();
    Eigen::Vector3f AlphaPv_((float)alpha_x->Value(), (float)alpha_y->Value(), (float)alpha_z->Value());
    Eigen::Matrix3f AlphaP_ = AlphaPv_.asDiagonal();
    Eigen::Vector3f Kpv_((float)Kp_x->Value(), (float)Kp_y->Value(), (float)Kp_z->Value());
    Eigen::Matrix3f Kp_ = Kpv_.asDiagonal();

    setFourierNN(W0_, (float)W1_fnn->Value(), (float)freq->Value(), (float)threshold->Value());
    setAdaptiveIntegralGain(Gamma0_, (float)Gamma1_int->Value());
    setISMCP((float)m->Value(), (float)g->Value(), AlphaP_, GammaP_, Kp_);
}

Eigen::Vector3f AFNNC::computeAFNNC(Eigen::Vector3f& nu, Eigen::Vector3f& sigma, float t, float delta_t) {
    return computeFourierNN(nu, t, delta_t) + computeIntegralTerm(nu, sigma, delta_t);
}

void AFNNC::UpdateFrom(const io_data *data) {

    float tactual=double(GetTime())/1000000000-t0;
    float Trs=0, tau_roll=0, tau_pitch=0, tau_yaw=0, Tr=0;

    if (T->Value() == 0) {
        delta_t = (float)(data->DataDeltaTime()) / 1000000000.;
    } else {
        delta_t = T->Value();
    }

    if (first_update == true) {
        delta_t = 0;
        first_update = false;
    }

    const Matrix* input = dynamic_cast<const Matrix*>(data);

    if (!input) {
        Warn("casting %s to Matrix failed\n",data->ObjectName().c_str(),TIME_INFINITE);
        return;
    }

    setCustomController();

    input->GetMutex();

    Eigen::Vector3f xie(input->ValueNoMutex(0, 0),input->ValueNoMutex(1, 0),input->ValueNoMutex(2, 0));
    Eigen::Vector3f xiep(input->ValueNoMutex(0, 1),input->ValueNoMutex(1, 1),input->ValueNoMutex(2, 1));

    Eigen::Vector3f xid(input->ValueNoMutex(0, 2),input->ValueNoMutex(1, 2),input->ValueNoMutex(2, 2));
    Eigen::Vector3f xidpp(input->ValueNoMutex(0, 4),input->ValueNoMutex(1, 4),input->ValueNoMutex(2, 4));
    Eigen::Vector3f xidppp(input->ValueNoMutex(0, 5),input->ValueNoMutex(1, 5),input->ValueNoMutex(2, 5));

    Eigen::Vector3f w(input->ValueNoMutex(0, 6),input->ValueNoMutex(1, 6),input->ValueNoMutex(2, 6));

    Eigen::Quaternionf q(input->ValueNoMutex(0, 7),input->ValueNoMutex(1, 7),input->ValueNoMutex(2, 7),input->ValueNoMutex(3, 7));

    input->ReleaseMutex();


    // Solve subactuation and get desired quaternion and angular velocity
    solveSubactuation(delta_t, q, xie, xiep, xid, xidpp, xidppp, w);

    Eigen::Quaternionf qd = getQd();
    Eigen::Vector3f wd = getWd();

    Eigen::Matrix3f Kdm = Eigen::Vector3f((float)Kd_roll->Value(), (float)Kd_pitch->Value(), (float)Kd_yaw->Value()).asDiagonal();
    Eigen::Matrix3f alphao = Eigen::Vector3f((float)alpha_roll->Value(), (float)alpha_pitch->Value(), (float)alpha_yaw->Value()).asDiagonal();


	// Compute attitude errors
    Eigen::Quaternionf qe = q*qd.conjugate();
	Eigen::Vector3f we = w - wd;

    Eigen::Quaternionf QdTqe3 = qd.conjugate()*qe*qd;

    Eigen::Vector3f nu = we + alphao*QdTqe3.vec();

    Eigen::Vector3f nu_t0 = 0.1*Eigen::Vector3f(1,1,1);

    Eigen::Vector3f nud = nu_t0*exp(-k->Value()*(tactual));

    Eigen::Vector3f nuq = nu-nud;

    Eigen::Vector3f tau = -Kdm*nuq;


    tau_roll = (float)tau(0)/km->Value();

    tau_pitch = (float)tau(1)/km->Value();

    tau_yaw = (float)tau(2)/km->Value();

    Tr = Trs/km->Value();

    tau_roll = -Sat(tau_roll,sat_r->Value());
    tau_pitch = -Sat(tau_pitch,sat_p->Value());
    tau_yaw = -Sat(tau_yaw,sat_y->Value());
    Tr = -Sat(Tr,sat_t->Value());

    // ToDo: Update mutex for output

    output->SetValue(0, 0, tau_roll);
    output->SetValue(1, 0, tau_pitch);
    output->SetValue(2, 0, tau_yaw);
    output->SetValue(3, 0, Tr);

    output->SetDataTime(data->DataTime());

    ProcessUpdate(output);
}

float AFNNC::Sat(float value, float borne) {
    if (value < -borne)
        return -borne;
    if (value > borne)
        return borne;
    return value;
}

void AFNNC::SetValues(Vector3Df xie, Vector3Df xiep, Vector3Df xid, Vector3Df xidpp, Vector3Df xidppp, Vector3Df w, Quaternion q){
    input->SetValue(0, 0, xie.x);
    input->SetValue(1, 0, xie.y);
    input->SetValue(2, 0, xie.z);

    input->SetValue(0, 1, xiep.x);
    input->SetValue(1, 1, xiep.y);
    input->SetValue(2, 1, xiep.z);

    input->SetValue(0, 2, xid.x);
    input->SetValue(1, 2, xid.y);
    input->SetValue(2, 2, xid.z);

    input->SetValue(0, 4, xidpp.x);
    input->SetValue(1, 4, xidpp.y);
    input->SetValue(2, 4, xidpp.z);

    input->SetValue(0, 5, xidppp.x);
    input->SetValue(1, 5, xidppp.y);
    input->SetValue(2, 5, xidppp.z);

    input->SetValue(0, 6, w.x);
    input->SetValue(1, 6, w.y);
    input->SetValue(2, 6, w.z);

    input->SetValue(0, 7, q.q0);
    input->SetValue(1, 7, q.q1);
    input->SetValue(2, 7, q.q2);
    input->SetValue(3, 7, q.q3);
}

} // end namespace filter
}