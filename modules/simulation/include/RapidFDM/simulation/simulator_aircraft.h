//
// Created by Hao Xu on 16/6/11.
//

#ifndef RAPIDFDM_SIMULATOR_AIRCRAFT_H
#define RAPIDFDM_SIMULATOR_AIRCRAFT_H

#include <RapidFDM/aerodynamics/aerodynamics.h>
#include <RapidFDM/control_system/control_system.h>
#include <PxRigidDynamic.h>
#include <PxFixedJoint.h>
#include <PxJoint.h>
#include <PxScene.h>
#include <RapidFDM/simulation/simulator_world.h>

using namespace physx;

namespace RapidFDM
{
    namespace Simulation
    {
        //! A simulator aircraft
        //! simulator aircraft is built by a aircraft node in airdynamics modules
        //! and a control system node
        //
        class SimulatorAircraft
        {
        protected:

            Aerodynamics::AircraftNode * aircraftNode = nullptr;
            ControlSystem::BaseController * baseController = nullptr;
//            std::
        public:
            SimulatorAircraft()
            {}
            SimulatorAircraft(  Aerodynamics::AircraftNode * _aircraftNode,
            ControlSystem::BaseController * _baseController):
                    aircraftNode(_aircraftNode),baseController(_baseController)
            {
                //Construct
                std::cerr << "Code didn't wrote :simulator_aircraft.h line 39" << std::endl;
                std::abort();
            }

            void construct_rigid_dynamics_from_aircraft(PxScene * pxScene);

        };
    }
}
#endif //RAPIDFDM_SIMULATOR_AEROCRAFT_H
