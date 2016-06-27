//
// Created by Hao Xu on 16/6/11.
//

#ifndef RAPIDFDM_SIMULATOR_WORLD_H
#define RAPIDFDM_SIMULATOR_WORLD_H

#ifndef NDEBUG
#define NDEBUG
#endif


#include <PxScene.h>
#include <PxPhysics.h>
using namespace physx;
namespace RapidFDM
{
    namespace Simulation
    {
        //!All realtime simulation should run in this
        static PxScene * pxScene = nullptr;
        PxPhysics * mPhysics = nullptr;
        float substep_deltatime = 0.001;
        class SimulatorWorld
        {

        public:
            void init(float substep_delatime);
            static void Step(float deltatime);
        };
    };
};
#endif //RAPIDFDM_SIMULATOR_WORLD_H
