//  created:    2024/11/18
//  filename:   AdaptiveFNN.h
//
//  author:     Alejandro TEVERA RUIZ
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: 1$
//
//  purpose:    Adaptive Fourier Neural Network controller
//
//
/*********************************************************************/

#ifndef CIRCLEFOLLOWER_H
#define CIRCLEFOLLOWER_H

#include <UavStateMachine.h>
#include "AFNNC.h"

namespace flair {
    namespace gui {
        class PushButton;
        class ComboBox;
        class Tab;
    }
    namespace filter {
        class TrajectoryGenerator2DCircle;
        class AFNNC;

    }
    namespace meta {
        class MetaVrpnObject;
    }
    namespace sensor {
        class TargetController;
    }
}

class AdaptiveFNN : public flair::meta::UavStateMachine {
    public:
        AdaptiveFNN(flair::sensor::TargetController *controller);
        ~AdaptiveFNN();

    private:

	enum class BehaviourMode_t {
            Default,
            PositionHold,
            Circle
        };

        BehaviourMode_t behaviourMode;
        bool vrpnLost;

        void VrpnPositionHold(void);//flight mode
        void StartCircle(void);
        void StopCircle(void);
        void ExtraSecurityCheck(void) override;
        void ExtraCheckPushButton(void) override;
        void ExtraCheckJoystick(void) override;
        const flair::core::AhrsData *GetOrientation(void) const override;
        void AltitudeValues(float &z,float &dz) const override;
        void PositionValues(flair::core::Vector2Df &pos_error,flair::core::Vector2Df &vel_error,float &yaw_ref);
        flair::core::AhrsData *GetReferenceOrientation(void) override;
        void SignalEvent(Event_t event) override;
        void ComputeCustomTorques(flair::core::Euler &torques);
        float ComputeCustomThrust(void);
        void fnn_controller(flair::core::Euler &torques);

        flair::filter::Pid *uX, *uY;

        flair::core::Vector2Df posHold;
        float yawHold;
        float thrust;

        flair::gui::Tab *settings, *lawTab2, *setupLawTab2;
        flair::gui::TabWidget *tabWidget2;


        flair::gui::PushButton *startCircle,*stopCircle,*positionHold;
        flair::gui::ComboBox *taskMode;
        flair::meta::MetaVrpnObject *targetVrpn,*uavVrpn;
        flair::filter::TrajectoryGenerator2DCircle *circle;
        flair::core::AhrsData *customReferenceOrientation,*customOrientation;

		flair::filter::AFNNC *customController;
};

#endif // CIRCLEFOLLOWER_H
