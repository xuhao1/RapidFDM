//
// Created by Hao Xu on 16/6/11.
//
#include <RapidFDM/simulation/simulator_world.h>

namespace RapidFDM
{
    namespace Simulation
    {
        void SimulatorWorld::init(float _substep_delatime)
        {
            substep_deltatime = _substep_delatime;
        }
    }
}