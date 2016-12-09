//
// Created by Hao Xu on 16/7/28.
//

#ifndef RAPIDFDM_MIXER_H
#define RAPIDFDM_MIXER_H

#include <RapidFDM/aerodynamics/nodes/bodys/aircraft_node.h>
using namespace RapidFDM::Aerodynamics;
namespace RapidFDM{
    namespace ControlSystem{
        typedef std::map<std::string,double> channel_mixer;
        //Mixer
        //Output: control values for components
        //Input: control
        class Mixer
        {
        protected:
            AircraftNode * aircraftNode = nullptr;
            std::map<std::string,channel_mixer> mixer_map;
        public:
            Mixer(rapidjson::Value & v);
            void run_mixer();
        };
    }
}

#endif //RAPIDFDM_MIXER_H
