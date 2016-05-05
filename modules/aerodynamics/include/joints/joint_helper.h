//
// Created by xuhao on 2016/5/5.
//

#ifndef RAPIDFDM_AERODYNAMICS_JOINT_HELPER_H
#define RAPIDFDM_AERODYNAMICS_JOINT_HELPER_H

#include <joints/Joint.h>
#include <joints/fixed_joint.h>
#include <iostream>
#include <stdio.h>
#include <fstream>

namespace RapidFDM {
    namespace Aerodynamics {
        class joint_helper {
        public:
            static Joint *create_joint_from_json(rapidjson::Value &v, std::map<std::string, Node *> nodeDB) {
                std::string type = fast_string(v, "type");
                if (type == "fixed") {
                    return new FixedJoint(v, nodeDB);
                }

                std::cerr << "Cannot parse Joint Type : " << type << std::endl;
                return nullptr;
            }

            static Joint *create_joint_from_json(std::string json, std::map<std::string, Node *> nodeDB) {
                rapidjson::Document document;
                document.Parse(json.c_str());
                return create_joint_from_json(document);
            }

            static Joint *create_joint_from_file(std::string file, std::map<std::string, Node *> nodeDB) {
                std::ifstream ifs(file);
                std::string content((std::istreambuf_iterator<char>(ifs)),
                                    (std::istreambuf_iterator<char>()));
                std::cout << "Json Content : \n" << content << std::endl;
                return create_joint_from_json(content);
            }
        };
    }
}

#endif //RAPIDFDM_JOINT_HELPER_H
