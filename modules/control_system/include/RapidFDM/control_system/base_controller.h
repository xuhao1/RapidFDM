//
// Created by Hao Xu on 16/6/11.
//

#ifndef RAPIDFDM_BASE_CONTROLLER_H
#define RAPIDFDM_BASE_CONTROLLER_H

#include <RapidFDM/aerodynamics/aerodynamics.h>

namespace RapidFDM
{
    namespace ControlSystem
    {
        class BaseController{
        public:
            Aerodynamics::AircraftNode * aircraftNode;
            BaseController(Aerodynamics::AircraftNode * _aircraftNode)
            {
                this->aircraftNode = _aircraftNode;
            }
            void control_step(float deltatime)
            {

            }
        };
    }
}

#endif //RAPIDFDM_BASE_CONTROLLER_H
