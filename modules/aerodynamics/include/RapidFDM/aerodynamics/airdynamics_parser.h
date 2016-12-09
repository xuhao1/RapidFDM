//
// Created by xuhao on 2016/5/3.
//

#ifndef RAPIDFDM_AIRDYNAMICS_PARSER_H
#define RAPIDFDM_AIRDYNAMICS_PARSER_H

#include <RapidFDM/aerodynamics/aerodynamics.h>
#include <string>

namespace RapidFDM {
    namespace Aerodynamics {
        class parser {
        protected:
            std::map<std::string,BaseNode * > nodes;
            std::map<std::string,BaseJoint *> joints;
            AircraftNode * aircraftNode;
        public:
            parser(std::string path);
            std::map<std::string,BaseNode * > get_nodes()
            {
                return nodes;
            };
            std::map<std::string,BaseJoint * > get_joints()
            {
                return joints;
            };
            AircraftNode * get_aircraft_node()
            {
                return aircraftNode;
            }
        };
    }
}

#endif //RAPIDFDM_AIRDYNAMICS_PARSER_H
