//
// Created by xuhao on 2016/5/4.
//

#ifndef RAPIDFDM_NODE_HELPER_H
#define RAPIDFDM_NODE_HELPER_H

#include <nodes/Node.h>
#include <nodes/bodys/aircraft_node.h>
#include <rapidjson/document.h>
#include <utils.h>

using namespace RapidFDM::Utils;

namespace RapidFDM {
    namespace Aerodynamics {
        class NodeHelper {
        public:
            static Node *create_node_from_json(rapidjson::Value &v) {
                std::string type = fast_string(v, "type");
                if (type == "aircraft") {
                    return new AircraftNode(v);
                }
            }
        };
    }
}

#endif //RAPIDFDM_NODE_HELPER_H
