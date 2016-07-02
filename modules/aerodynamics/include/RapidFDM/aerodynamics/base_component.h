#ifndef __RAPIDFDM_AERODYNAMIC_BASE_COMPONENT_H__
#define __RAPIDFDM_AERODYNAMIC_BASE_COMPONENT_H__

#include <Eigen/Eigen>
#include <iostream>
#include <RapidFDM/utils.h>
#include <stdlib.h>
#include <stdio.h>
#include <map>
#include <string>
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
        public:
            BaseComponent()
            {
            }

            BaseComponent(rapidjson::Value &v)
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

            virtual std::map<std::string,double> get_control_axis()
            {
                return control_axis;
            };

            virtual void set_control_value(std::string name, double v)
            {
                this->control_axis[name] = v;
            }

            virtual void set_internal_state(std::string name,double v)
            {
                this->internal_states[name] = v;
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

        };
    }
}

#endif