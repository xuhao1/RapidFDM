//
// Created by xuhao on 2016/5/4.
//

#ifndef RAPIDFDM_NODE_HELPER_H
#define RAPIDFDM_NODE_HELPER_H

#include <nodes/Node.h>
#include <nodes/bodys/aircraft_node.h>
#include <nodes/wings/wing_node.h>
#include <rapidjson/document.h>
#include <utils.h>
#include <iostream>
#include <fstream>

using namespace RapidFDM::Utils;

namespace RapidFDM {
    namespace Aerodynamics {
        class NodeHelper {
        public:
            static Node *create_node_from_json(rapidjson::Value &v) {
                std::string type = fast_string(v, "type");
                if (type == "aircraft") {
                    printf("Parse Aircraft node\n");
                    return new AircraftNode(v);
                }
                if (type == "wing") {
                    printf("Parse Wing node\n");
                    return new WingNode(v);
                }
                std::cerr << "Cannot parse Node Type : " << type << std::endl;
                return nullptr;
            }

            static Node *create_node_from_json(std::string json) {
                rapidjson::Document document;
                document.Parse(json.c_str());
                return create_node_from_json(document);
            }

            static Node *create_node_from_file(std::string file) {
                std::ifstream ifs(file);
                std::string content((std::istreambuf_iterator<char>(ifs)),
                                    (std::istreambuf_iterator<char>()));
                std::cout << "Json Content : \n" << content << std::endl;
                return create_node_from_json(content);
            }
        };
    }
}

#endif //RAPIDFDM_NODE_HELPER_H
