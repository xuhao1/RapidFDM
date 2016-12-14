//
// Created by xuhao on 2016/12/14.
//

#ifndef RAPIDFDM_SIMULATION_HIL_ADAPTER_H
#define RAPIDFDM_SIMULATION_HIL_ADAPTER_H

#include "simulator_aircraft.h"

namespace RapidFDM
{
    namespace Simulation{
        class simulation_hil_adapter{
        protected:
            SimulatorAircraft * sim_air = nullptr;
        public:
            simulation_hil_adapter(SimulatorAircraft * _sim_air):
                    sim_air(_sim_air)
            {

            }

            virtual bool enableSimulation() = 0;
        };
    }
}

#endif //RAPIDFDM_SIMULATION_HIL_ADAPTER_H
