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
#include <vector>

using namespace physx;

namespace RapidFDM
{
    namespace Simulation
    {
        struct node_rigid
        {
            PxRigidBody *rigid;
            Aerodynamics::Node *node;
        };
        struct joint_Joint
        {
            PxJoint *joint_physx;
            Aerodynamics::Joint *joint_aerodynamics;
        };

        //! A simulator aircraft
        //! simulator aircraft is built by a aircraft node in airdynamics modules
        //! and a control system node
        //
        class SimulatorAircraft
        {
        protected:

            Aerodynamics::AircraftNode *aircraftNode = nullptr;
            ControlSystem::BaseController *baseController = nullptr;
//            std::
        public:
            SimulatorAircraft()
            {
            }

            SimulatorAircraft(Aerodynamics::AircraftNode *_aircraftNode,
                              ControlSystem::BaseController *_baseController) :
                    aircraftNode(_aircraftNode), baseController(_baseController)
            {
                //Construct
                std::cerr << "Code didn't wrote :simulator_aircraft.h line 39" << std::endl;
                std::abort();
            }

            static void dfs_create_rigids(
                    Aerodynamics::Node * root,
                    std::vector<node_rigid *> &nodes,
                    std::vector<joint_Joint *> &joints
            );

            void construct_rigid_dynamics_from_aircraft(PxScene *pxScene);

        };
    }
}
#endif //RAPIDFDM_SIMULATOR_AEROCRAFT_H
