//
// Created by Hao Xu on 16/6/11.
//

#ifndef RAPIDFDM_SIMULATOR_WORLD_H
#define RAPIDFDM_SIMULATOR_WORLD_H

#include <RapidFDM/simulation/simulator_aircraft.h>

#ifndef NDEBUG
#define NDEBUG
#endif

#include <PxScene.h>
#include <PxPhysics.h>
#include <PxSceneDesc.h>
#include "PxPhysicsAPI.h"

#undef NDEBUG
using namespace physx;
namespace RapidFDM
{
    namespace Simulation
    {
        //!All realtime simulation should run in this

        class SimulatorWorld
        {
            PxDefaultErrorCallback gDefaultErrorCallback;
            PxDefaultAllocator gDefaultAllocatorCallback;
            PxSimulationFilterShader gDefaultFilterShader = PxDefaultSimulationFilterShader;
            void *mScratchBlock;
            physx::PxFoundation *mFoundation;
            PxSceneDesc *sceneDesc;

        public:
            SimulatorWorld(float dt)
            {
                init(substep_deltatime);
            }
            PxScene *pxScene = nullptr;
            PxPhysics *mPhysics = nullptr;
            float substep_deltatime = 0.001;

            void init(float substep_delatime);

            void Step(float deltatime);

            SimulatorAircraft * create_aircraft(Aerodynamics::AircraftNode *_aircraftNode,
                              ControlSystem::BaseController *_baseController)
            {
                return new SimulatorAircraft(_aircraftNode,_baseController,this);
            }
        };
    };
};
#endif //RAPIDFDM_SIMULATOR_WORLD_H
