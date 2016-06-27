//
// Created by xuhao on 2016/5/5.
//

#ifndef RAPIDFDM_AERODYNAMICS_JOINT_HELPER_H
#define RAPIDFDM_AERODYNAMICS_JOINT_HELPER_H

#include <RapidFDM/aerodynamics/joints/Joint.h>
#include <RapidFDM/aerodynamics/joints/fixed_joint.h>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <map>

namespace RapidFDM {
    namespace Aerodynamics {
        class JointHelper {
        public:
            static Joint *create_joint_from_json(rapidjson::Value &v, std::map<std::string, Node *> nodeDB) {
                std::string type = fast_string(v, "type");
                if (type == "fixed") {
                    printf("Parse fixed joint\n");
                    return new FixedJoint(v, nodeDB);
                }

                std::cerr << "Cannot parse Joint Type : " << type << std::endl;
                return nullptr;
            }

            static Joint *create_joint_from_json(std::string json, std::map<std::string, Node *> nodeDB) {
                rapidjson::Document document;
                document.Parse(json.c_str());
                return create_joint_from_json(document, nodeDB);
            }

            static Joint *create_joint_from_file(std::string file, std::map<std::string, Node *> nodeDB) {
                std::ifstream ifs(file);
                std::string content((std::istreambuf_iterator<char>(ifs)),
                                    (std::istreambuf_iterator<char>()));
                std::cout << "Json Content : \n" << content << std::endl;
                return create_joint_from_json(content, nodeDB);
            }
            static std::map<std::string,Joint * > scan_joint_folder(std::string path,std::map<std::string, Node *> nodeDB)
            {
                std::map<std::string,Joint *> jointDB;
                std::vector<std::string> file_list = get_file_list(path);
                for (std::string file_path : file_list) {
                    printf("Scan file : %s\n", file_path.c_str());
                    Joint *tmp = create_joint_from_file(file_path,nodeDB);
                    if (tmp != nullptr) {
                        jointDB[tmp->getUniqueID()] = tmp;
                    }
                }
                return jointDB;
            };
        };
    }
}

#endif //RAPIDFDM_JOINT_HELPER_H
