//
// Created by xuhao on 2017/3/23.
//

#ifndef RAPIDFDM_TCC_DYNAMICS_CONTROLLER_H
#define RAPIDFDM_TCC_DYNAMICS_CONTROLLER_H

#include <RapidFDM/control_system/base_controller.h>
#include "mat.h"

extern "C" {
#include "L1Step2ndRoll.h"
};


namespace RapidFDM
{
    namespace ControlSystem{
        class so3_adaptive_controller: public BaseController
        {
        public:

            AdaptiveSysT sys = {0};
            AdaptiveCtrlT ctrlRoll = {0};
            MATFile *pmat;

            std::vector<AdaptiveCtrlT> ctrl_log;
            std::vector<AdaptiveSysT> sys_log;

            so3_adaptive_controller(Aerodynamics::AircraftNode * _aircraftNode);
            virtual void control_step(float deltatime) override ;

            void save_data_file();

        };
    }
}

#endif //RAPIDFDM_TCC_DYNAMICS_CONTROLLER_H
