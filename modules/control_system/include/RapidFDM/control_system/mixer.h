//
// Created by Hao Xu on 16/7/28.
//

#ifndef RAPIDFDM_MIXER_H
#define RAPIDFDM_MIXER_H

#include <RapidFDM/aerodynamics/nodes/bodys/aircraft_node.h>
using namespace RapidFDM::Aerodynamics;
namespace RapidFDM{
    namespace ControlSystem{
        //Mixer
        //Output: control values for components
        //Input: control
        class Mixer
        {
        protected:
            AircraftNode * aircraftNode = nullptr;
        public:
            Mixer()
            {

            }
        };
    }
}

#endif //RAPIDFDM_MIXER_H
