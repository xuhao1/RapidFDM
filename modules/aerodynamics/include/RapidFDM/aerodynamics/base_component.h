#ifndef __RAPIDFDM_AERODYNAMIC_BASE_COMPONENT_H__
#define __RAPIDFDM_AERODYNAMIC_BASE_COMPONENT_H__

#include <Eigen/Eigen>
#include <iostream>
#include <RapidFDM/utils.h>
#include <stdlib.h>
#include <stdio.h>
#include <map>
#include <string>

using namespace RapidFDM::Utils;

namespace RapidFDM {
    namespace Aerodynamics {
        class BaseComponent {
        protected:
            std::string name;
            std::string unique_id;
            std::map<std::string,double> inertial_states;
            std::map<std::string,double> control_axis;

            double time = 0;
//            std::vector<std::string> control_frame;
        public:
            BaseComponent() { }

            BaseComponent(rapidjson::Value &v) {
                this->name = fast_string(v, "name");
                std::string _id = fast_string(v,"id");
                if (_id == "")
                {
                    char buffer[8] = {0};
                    itoa(rand()%100000000,buffer,0);
                    _id  = std::string(buffer);
                }

                this->unique_id = this->name + "_" + this->unique_id;

            }


            virtual Eigen::Vector3d get_ground_velocity() {
                std::abort();
            }

            virtual Eigen::Vector3d get_angular_velocity() {
                std::abort();
            }

            virtual Eigen::Vector3d get_ground_air_speed()
            {
                std::abort();
            }

            virtual Eigen::Affine3d get_ground_transform() {
                std::abort();
            }


            virtual Eigen::Quaterniond get_ground_attitude() {
                std::abort();
            }


            //!Air velocity relative to node in local transform,shall consider velocity from angular speed at center point
            //
            virtual Eigen::Vector3d get_air_velocity() {
                std::abort();
            }


            virtual Eigen::Affine3d get_body_transform() {
                std::abort();
            }

            virtual std::map<std::string,double>  get_inertial_states () {
                return this->inertial_states;
            };

            virtual std::vector<std::string> get_control_frame ()
            {
                std::vector<std::string> res;
                for (auto s : control_axis)
                {
                    res.push_back(s.first);
                }
                return res;
            }

            virtual void set_control_value(std::string name,double v)
            {
                this->control_axis[name] = v;
            }

            virtual void iter_inertial_state(double deltatime)
            {
            }

            virtual void brief() { }

            std::string getName() {
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