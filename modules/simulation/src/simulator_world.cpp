//
// Created by Hao Xu on 16/6/11.
//
#include <RapidFDM/simulation/simulator_world.h>
#include <cassert>

#ifndef NDEBUG
#define NDEBUG
#endif

#include <PxSceneDesc.h>
#include "PxPhysicsAPI.h"


namespace RapidFDM
{
    namespace Simulation
    {
        static PxDefaultErrorCallback gDefaultErrorCallback;
        static PxDefaultAllocator gDefaultAllocatorCallback;
        static PxSimulationFilterShader gDefaultFilterShader = PxDefaultSimulationFilterShader;
        static void *mScratchBlock;
        static physx::PxFoundation *mFoundation;
        static PxSceneDesc *sceneDesc;

        void SimulatorWorld::init(float _substep_delatime)
        {
            printf("Trting to init simulator\n");
            substep_deltatime = _substep_delatime;
            mFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gDefaultAllocatorCallback, gDefaultErrorCallback);
            assert(mFoundation != nullptr);
            auto mProfileZoneManager = &PxProfileZoneManager::createProfileZoneManager(mFoundation);
            assert(mProfileZoneManager != nullptr);
            bool recordMemoryAllocations = true;
            mPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation,
                                       PxTolerancesScale(), recordMemoryAllocations, mProfileZoneManager);
            if (!mPhysics)
                printf("PxCreatePhysics failed!");

            sceneDesc = new PxSceneDesc(mPhysics->getTolerancesScale());

            sceneDesc->gravity = PxVec3(0.0f, 0, -9.8f);

            //customizeSceneDesc(sceneDesc);
            if (!sceneDesc->cpuDispatcher) {
                auto mCpuDispatcher = PxDefaultCpuDispatcherCreate(1);
                assert(mCpuDispatcher != nullptr);
                sceneDesc->cpuDispatcher = mCpuDispatcher;
            }

            if (!sceneDesc->filterShader) {
                sceneDesc->filterShader = gDefaultFilterShader;
            }
            sceneDesc->flags |= PxSceneFlag::eENABLE_CCD;
            pxScene = mPhysics->createScene(*sceneDesc);

            printf("Init physics successful!\n");

        }
    }
}