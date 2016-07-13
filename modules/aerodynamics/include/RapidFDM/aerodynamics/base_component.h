#ifndef __RAPIDFDM_AERODYNAMIC_BASE_COMPONENT_H__
#define __RAPIDFDM_AERODYNAMIC_BASE_COMPONENT_H__

#include <Eigen/Eigen>
#include <iostream>
#include <RapidFDM/utils.h>
#include <stdlib.h>
#include <stdio.h>
#include <map>
#include <string>
#include <rapidjson/document.h>
#include "FlyingData.h"

using namespace RapidFDM::Utils;

namespace RapidFDM
{
    namespace Aerodynamics
    {
        class BaseComponent
        {
        protected:
            std::string name;
            std::string unique_id;
            std::map<std::string, double> internal_states;
            std::map<std::string, double> control_axis;

            double time = 0;
//            std::vector<std::string> control_frame;
            rapidjson::Document source_document;
        public:
            BaseComponent()
            {
            }

            BaseComponent(const rapidjson::Value &v)
            {
                assert(v.IsObject());
                this->name = fast_string(v, "name");
                std::string _id = fast_string(v, "id");
                if (_id == "") {
                    char buffer[9] = {0};
                    sprintf(buffer, "%d", rand() % 100000000);
                    _id = std::string(buffer);
                }

                this->unique_id = this->name + "_" + _id;
                this->source_document.CopyFrom(v, source_document.GetAllocator());
            }


            virtual Eigen::Vector3d get_ground_velocity()
            {
                std::abort();
            }

            virtual Eigen::Vector3d get_angular_velocity()
            {
                std::abort();
            }

            virtual Eigen::Affine3d get_ground_transform()
            {
                std::abort();
            }


            virtual Eigen::Quaterniond get_ground_attitude()
            {
                std::abort();
            }


            virtual Eigen::Affine3d get_body_transform()
            {
                std::abort();
            }

            virtual std::map<std::string, double> get_internal_states()
            {
                return this->internal_states;
            };

            virtual std::vector<std::string> get_control_frame()
            {
                std::vector<std::string> res;
                for (auto s : control_axis) {
                    res.push_back(s.first);
                }
                return res;
            }

            virtual std::map<std::string, double> get_control_axis()
            {
                return control_axis;
            };

            virtual int set_control_value(std::string name, double v)
            {
                if (this->control_axis.find(name) != this->control_axis.end()) {
                    this->control_axis[name] = v;
                    return 0;
                }
                return -1;
            }

            virtual int set_internal_state(std::string name, double v)
            {
                if (this->internal_states.find(name) != this->internal_states.end()) {
                    this->internal_states[name] = v;
                    return 0;
                }
                return -1;
            }

            virtual void iter_internal_state(double deltatime)
            {
            }

            virtual void brief()
            {
            }


            std::string getName()
            {
                return this->name;
            }

            std::string getUniqueID()
            {
                return this->unique_id;
            }

        public:
            virtual const rapidjson::Value &getJsonDefine()
            {
                return this->source_document;
            }

        };
    }
}

#endif