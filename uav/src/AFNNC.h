//
// Created by ateveraz on 19/11/24.
//
#ifndef AFNNC_H
#define AFNNC_H

#include <iostream>
#include "FourierNN.h"
#include "AdapIntegralGain.h"
#include "ISMCP.h"
#include <Matrix.h>
#include <Layout.h>
#include <LayoutPosition.h>
#include <DoubleSpinBox.h>
#include <Vector3D.h>
#include <Vector3DSpinBox.h>
#include <SpinBox.h>
#include <GroupBox.h>
#include <Quaternion.h>
#include <Euler.h>
#include <ControlLaw.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <Tab.h>

using namespace std;
using std::string;
using namespace flair::core;
using namespace flair::gui;

namespace flair {
	namespace core {
        class Matrix;
        class io_data;
    }
    namespace gui {
        class LayoutPosition;
        class Tab;
        class DoubleSpinBox;
        class CheckBox;
        class Label;
    }
}

namespace flair {
namespace filter {
class AFNNC : public ControlLaw, public FourierNN, public AdapIntegralGain, public ISMCP {
    public:
        AFNNC(const flair::gui::LayoutPosition *position, std::string name);
        ~AFNNC();

        void setCustomController();
        void setISMC(const float m_, const float g_, Eigen::Matrix3f alphap_, Eigen::Matrix3f gammap_, Eigen::Matrix3f Kpm_);
        void reset();

           /*!
  * \brief Set input values
  *
  * \param xie Error de posicion
  * \param xiep Error de velocidad
  * \param xid  Posicion deseada
  * \param xidpp Aceleracion deseada
  * \param xidppp Jerk deseado
  * \param w Velocidad angular
  * \param q Cuaternio de orientacion
  */
    	void SetValues(flair::core::Vector3Df xie, flair::core::Vector3Df xiep, flair::core::Vector3Df xid,
                    flair::core::Vector3Df xidpp, flair::core::Vector3Df xidppp, flair::core::Vector3Df w, flair::core::Quaternion q);

        void UpdateFrom(const flair::core::io_data *data);

        void cartesian2configuration(Eigen::Quaternionf &qd, Eigen::Vector3f &wfd, const float delta_t, Eigen::Quaternionf q, Eigen::Vector3f xie, Eigen::Vector3f xiep, Eigen::Vector3f xid, Eigen::Vector3f xidpp, Eigen::Vector3f xidppp);

        flair::core::Time t0;

    private:
		std::chrono::high_resolution_clock::time_point previous_chrono_time;
		bool firstUpdate;

		flair::core::Matrix *exp_data; // Description Matrix

        float Sat(float value, float borne);

		Eigen::Vector3f computeAFNNC(Eigen::Vector3f& nu, Eigen::Vector3f& sigma, float t, float delta_t);

        // Variables de GUI
        flair::gui::DoubleSpinBox *T, *k1, *k2, *gamma, *k, *sat_r, *sat_p, *sat_y, *sat_t, *m, *g, *km, *freq, *p, *threshold;
        flair::gui::DoubleSpinBox *alpha_roll, *alpha_pitch, *alpha_yaw;
        flair::gui::DoubleSpinBox *W0_roll, *W0_pitch, *W0_yaw, *W1_fnn;
        flair::gui::DoubleSpinBox *Gamma0_roll, *Gamma0_pitch, *Gamma0_yaw, *Gamma1_int;
        flair::gui::DoubleSpinBox *Kd_roll, *Kd_pitch, *Kd_yaw;

        // SMCP gains
        flair::gui::DoubleSpinBox *alpha_x, *alpha_y, *alpha_z;
        flair::gui::DoubleSpinBox *gamma_x, *gamma_y, *gamma_z;
        flair::gui::DoubleSpinBox *Kp_x, *Kp_y, *Kp_z;

        float delta_t;

        bool first_update;
};
}
}

#endif // AFNNC_H