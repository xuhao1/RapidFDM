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
            std::map<std::string,Node * > nodes;
            std::map<std::string,Joint *> joints;
            AircraftNode * aircraftNode;
        public:
            parser(std::string path);
            std::map<std::string,Node * > get_nodes()
            {
                return nodes;
            };
            std::map<std::string,Joint * > get_joints()
            {
                return joints;
            };
            AircraftNode * get_aircraft_node();
        };
    }
}

#endif //RAPIDFDM_AIRDYNAMICS_PARSER_H
