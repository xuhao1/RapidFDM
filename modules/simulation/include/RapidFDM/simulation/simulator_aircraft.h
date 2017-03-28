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
            std::map<BaseNode *, PxRigidDynamic *> nodes;
            std::map<BaseJoint *, PxJoint *> joints;
            PxScene *pxScene = nullptr;
            PxPhysics *mPhysics = nullptr;
//            std::
            int count = 0;
        public:
            Eigen::Vector3d gAcc = Eigen::Vector3d(0,0,0);
            SimulatorAircraft(PxTransform init_trans = PxTransform::createIdentity())
            {
            }

            SimulatorAircraft(
                    Aerodynamics::AircraftNode *_aircraftNode,
                    SimulatorWorld *_simulator,
                    PxTransform init_trans = PxTransform::createIdentity(),
                    double init_speed = 0
            );

            void dfs_create_rigids(
                    Aerodynamics::BaseNode *root,
                    std::map<BaseNode *, PxRigidDynamic *> &nodes,
                    std::map<BaseJoint *, PxJoint *> &joints,
                    PxRigidDynamic *root_rigid
            );

            void construct_rigid_dynamics_from_aircraft();

            PxRigidDynamic *construct_rigid(Aerodynamics::BaseNode *node);

            PxRigidDynamic * construct_rigid_aircraft();

            virtual void fetch_forces_torques_from_aerodynamics();

            PxJoint *construct_joint(
                    Aerodynamics::BaseNode *root,
                    Aerodynamics::BaseJoint *joint,
                    PxRigidDynamic *root_rigid,
                    PxRigidDynamic *child_rigid
            );
            void reset_aircraft(
                    PxTransform init_trans = PxTransform::createIdentity(),
                    double init_speed = 0
            );
            void update_states_from_physx();

            AircraftNode * get_aircraft_node()
            {
                return aircraftNode;
            }

        };
    }
}
#endif //RAPIDFDM_SIMULATOR_AEROCRAFT_H
