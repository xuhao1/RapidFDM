//
// Created by xuhao on 2016/5/4.
//

#ifndef RAPIDFDM_NODE_HELPER_H
#define RAPIDFDM_NODE_HELPER_H

#include <RapidFDM/aerodynamics/nodes/base_node.h>
#include <RapidFDM/aerodynamics/nodes/bodys/aircraft_node.h>
#include <RapidFDM/aerodynamics/nodes/wings/wing_node.h>
#include <RapidFDM/aerodynamics/nodes/engines/easy_propeller.h>
#include <rapidjson/document.h>
#include <RapidFDM/utils.h>
#include <iostream>
#include <fstream>
#include "Nodes.h"

using namespace RapidFDM::Utils;

namespace RapidFDM {
    namespace Aerodynamics {
        class NodeHelper {
        public:
            static BaseNode *create_node_from_json(const rapidjson::Value &v) {
                assert(v.IsObject());
                std::string type = fast_string(v, "type");
                if (type == "aircraft") {
                    return new AircraftNode(v);
                }
                if (type == "wing") {
                    return new WingNode(v);
                }
                if (type == "easy_propeller")
                {
                    return new EasyPropellerNode(v);
                }
                std::cerr << "Cannot parse BaseNode Type : " << type << std::endl;
                return nullptr;
            }

            static BaseNode *create_node_from_json(std::string json) {
                rapidjson::Document document;
                document.Parse(json.c_str());
                return create_node_from_json(document);
            }

            static BaseNode *create_node_from_file(std::string file) {
                std::ifstream ifs(file);
                std::string content((std::istreambuf_iterator<char>(ifs)),
                                    (std::istreambuf_iterator<char>()));
                return create_node_from_json(content);
            }

            //This function return a list of nodes
            static std::map<std::string, BaseNode *> scan_node_folder(std::string path) {
                printf("Scanning node folder %s \n",path.c_str());
                std::map<std::string, BaseNode *> nodeDB;
                std::vector<std::string> file_list = get_file_list(path);
                for (std::string file_path : file_list) {
                    BaseNode *tmp = create_node_from_file(file_path);
                    if (tmp != nullptr) {
                        nodeDB[tmp->getUniqueID()] = tmp;
                    }
                }
                printf("Scan folder: %s finish\n",path.c_str());
                return nodeDB;
            };
        };
    }
}

#endif //RAPIDFDM_NODE_HELPER_H
