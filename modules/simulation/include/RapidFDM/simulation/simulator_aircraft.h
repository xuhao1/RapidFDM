//
// Created by Hao Xu on 16/6/11.
//

#ifndef RAPIDFDM_SIMULATOR_AIRCRAFT_H
#define RAPIDFDM_SIMULATOR_AIRCRAFT_H


#include <RapidFDM/aerodynamics/aerodynamics.h>
#include <RapidFDM/control_system/control_system.h>

#ifndef NDEBUG
#define NDEBUG
#endif

#include <PxRigidDynamic.h>
#include <PxFixedJoint.h>
#include <PxJoint.h>
#include <PxScene.h>

#undef NDEBUG

#include <vector>

using namespace physx;
using namespace RapidFDM::Aerodynamics;


namespace RapidFDM
{
    namespace Simulation
    {

        class SimulatorWorld;


        //! A simulator aircraft
        //! simulator aircraft is built by a aircraft node in airdynamics modules
        //! and a control system node
        //
        class SimulatorAircraft
        {
        protected:

            Aerodynamics::AircraftNode *aircraftNode = nullptr;
            ControlSystem::BaseController *baseController = nullptr;
            std::map<BaseNode *, PxRigidBody *> nodes;
            std::map<BaseJoint *, PxJoint *> joints;
            PxScene *pxScene = nullptr;
            PxPhysics *mPhysics = nullptr;
//            std::
        public:
            SimulatorAircraft()
            {
            }

            SimulatorAircraft(
                    Aerodynamics::AircraftNode *_aircraftNode,
                    ControlSystem::BaseController *_baseController,
                    SimulatorWorld *_simulator
            );

            void dfs_create_rigids(
                    Aerodynamics::BaseNode *root,
                    std::map<BaseNode *, PxRigidBody *> &nodes,
                    std::map<BaseJoint *, PxJoint *> &joints,
                    PxRigidBody *root_rigid
            );

            void construct_rigid_dynamics_from_aircraft();

            PxRigidBody *construct_rigid(Aerodynamics::BaseNode *node);

            PxJoint *construct_joint(
                    Aerodynamics::BaseNode *root,
                    Aerodynamics::BaseJoint *joint,
                    PxRigidBody *root_rigid,
                    PxRigidBody *child_rigid
            );

            void update_states_from_physx();

        };
    }
}
#endif //RAPIDFDM_SIMULATOR_AEROCRAFT_H
