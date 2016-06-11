//
// Created by Hao Xu on 16/6/11.
//

#ifndef RAPIDFDM_SIMULATOR_WORLD_H
#define RAPIDFDM_SIMULATOR_WORLD_H
namespace RapidFDM
{
    namespace Simulation
    {
        //!All realtime simulation should run in this
        class SimulatorWorld
        {
        public:
            //!Construct a simulator world
            //!\param substep time for internal running
            SimulatorWorld(float substep_deltatime);
            void Step(float deltatime);
        };
    };
};
#endif //RAPIDFDM_SIMULATOR_WORLD_H
