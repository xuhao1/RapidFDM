//
// Created by Hao Xu on 16/6/11.
//

#ifndef RAPIDFDM_SIMULATOR_AIRCRAFT_H
#define RAPIDFDM_SIMULATOR_AIRCRAFT_H

#ifndef NDEBUG
#define NDEBUG
#endif

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
            PxRigidBody *rigid = nullptr;
            Aerodynamics::Node *node = nullptr;

            node_rigid()
            {
            }

            node_rigid(PxRigidBody *_rigid, Aerodynamics::Node *_node) :
                    rigid(_rigid), node(_node)
            {
            }
        };

        struct joint_Joint
        {
            PxJoint *joint_physx = nullptr;
            Aerodynamics::Joint *joint_aerodynamics = nullptr;

            joint_Joint()
            {
            }

            joint_Joint(PxJoint *_joint_physx, Aerodynamics::Joint *_joint_Aero) :
                    joint_physx(_joint_physx), joint_aerodynamics(_joint_Aero)
            {
            }
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
            std::vector<node_rigid *> nodes;
            std::vector<joint_Joint *> joints;
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
//                std::cerr << "Code didn't wrote :simulator_aircraft.h line 39" << std::endl;
//                std::abort();
                construct_rigid_dynamics_from_aircraft();
            }

            static void dfs_create_rigids(
                    Aerodynamics::Node *root,
                    std::vector<node_rigid *> &nodes,
                    std::vector<joint_Joint *> &joints,
                    PxRigidBody *root_rigid
            );

            void construct_rigid_dynamics_from_aircraft();

            static PxRigidBody *construct_rigid(Aerodynamics::Node *node);

            static PxJoint *construct_joint(
                    Aerodynamics::Node *root,
                    Aerodynamics::Joint *joint,
                    PxRigidBody *root_rigid,
                    PxRigidBody *child_rigid
            );

        };
    }
}
#endif //RAPIDFDM_SIMULATOR_AEROCRAFT_H
