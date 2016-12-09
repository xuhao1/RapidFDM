//
// Created by Hao Xu on 16/6/11.
//
#include <RapidFDM/simulation/simulator_world.h>
#ifndef NDEBUG
#define NDEBUG
#endif

#include <PxSceneDesc.h>
#include "PxPhysicsAPI.h"

#undef NDEBUG

namespace RapidFDM
{
    namespace Simulation
    {

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

            assert(pxScene!= nullptr);
            assert(mPhysics!= nullptr);

            PxMaterial * aMaterial =  mPhysics->createMaterial(0.5f, 0.8f, 0.1f);    //static friction, dynamic friction, restitution
            PxRigidStatic * plane =  PxCreatePlane(*mPhysics, PxPlane(PxVec3(0,0,1), 0), *aMaterial);
            pxScene->addActor(*plane);

            printf("Init physics successful!\n");

        }

        void SimulatorWorld::Step(float deltatime)
        {
            for (int j=0; j<deltatime/substep_deltatime; j++)
            {
                pre_sim_setup();
                //TODO:
                //Move this
                aircraft->get_aircraft_node()->iter_internal_state(substep_deltatime/1000);
                pxScene->simulate(substep_deltatime/1000);
                pxScene->fetchResults(true);
            }
        }

        void SimulatorWorld::pre_sim_setup()
        {
            assert(aircraft!= nullptr);
            aircraft->update_states_from_physx();
            aircraft->fetch_forces_torques_from_aerodynamics();
        }
    }
}