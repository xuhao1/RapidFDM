//
// Created by xuhao on 2017/3/23.
//

#ifndef RAPIDFDM_SIMULATION_SITL_ADAPTER_H
#define RAPIDFDM_SIMULATION_SITL_ADAPTER_H
#include "simulation_hil_adapter.h"
#include <RapidFDM/control_system/base_controller.h>
#include <string>

using namespace RapidFDM::ControlSystem;
namespace RapidFDM {
    namespace Simulation {
        class simulation_sitl_adapter : public simulation_hil_adapter {
        private:

            float RcA = 0, RcE = 0, RcR = 0, RcT = 0;
            float pwm[8] = {0};

            BaseController * controller;

        protected:
            virtual void tick_func(float dt,long tick) override;

        public:
            simulation_sitl_adapter(SimulatorAircraft *simulatorAircraft,BaseController * _controller,float dt = 5);
            void handle_chn_from_joystick(float * pwm,int num);
            virtual bool enable_simulation() override;
            virtual void push_json_to_app(rapidjson::Document & d) override ;
        };
    }
}


#endif //RAPIDFDM_SIMULATION_SITL_ADAPTER_H
