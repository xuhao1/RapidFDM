//
// Created by xuhao on 2016/5/5.
//

#ifndef RAPIDFDM_AERODYNAMICS_JOINT_HELPER_H
#define RAPIDFDM_AERODYNAMICS_JOINT_HELPER_H

#include <RapidFDM/aerodynamics/joints/base_joint.h>
#include <RapidFDM/aerodynamics/joints/fixed_joint.h>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <map>

namespace RapidFDM {
    namespace Aerodynamics {
        class JointHelper {
        public:
            static BaseJoint *create_joint_from_json(const rapidjson::Value &v, std::map<std::string, BaseNode *> nodeDB) {
                assert(v.IsObject());
                std::string type = fast_string(v, "type");
                if (type == "fixed") {
                    printf("Parse fixed joint\n");
                    return new FixedJoint(v, nodeDB);
                }

                std::cerr << "Cannot parse BaseJoint Type : " << type << std::endl;
                return nullptr;
            }

            static BaseJoint *create_joint_from_json(std::string json, std::map<std::string, BaseNode *> nodeDB) {
                rapidjson::Document document;
                document.Parse(json.c_str());
                return create_joint_from_json(document, nodeDB);
            }

            static BaseJoint *create_joint_from_file(std::string file, std::map<std::string, BaseNode *> nodeDB) {
                std::ifstream ifs(file);
                std::string content((std::istreambuf_iterator<char>(ifs)),
                                    (std::istreambuf_iterator<char>()));
//                std::cout << "Json Content : \n" << content << std::endl;
                return create_joint_from_json(content, nodeDB);
            }
            static std::map<std::string,BaseJoint * > scan_joint_folder(std::string path,std::map<std::string, BaseNode *> nodeDB)
            {
                printf("Scanning joint foilder %s \n",path.c_str());
                std::map<std::string,BaseJoint *> jointDB;
                std::vector<std::string> file_list = get_file_list(path);
                for (std::string file_path : file_list) {
                    printf("Scan file : %s\n", file_path.c_str());
                    BaseJoint *tmp = create_joint_from_file(file_path,nodeDB);
                    if (tmp != nullptr) {
                        jointDB[tmp->getUniqueID()] = tmp;
                    }
                }
                printf("Scan folder: %s finish\n",path.c_str());
                return jointDB;
            };
        };
    }
}

#endif //RAPIDFDM_JOINT_HELPER_H
