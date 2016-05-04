#ifndef __RAPIDFDM_AERODYNAMIC_BASE_COMPONENT_H__
#define __RAPIDFDM_AERODYNAMIC_BASE_COMPONENT_H__

#include <Eigen/Eigen>
#include <iostream>
#include <utils.h>

using namespace RapidFDM::Utils;

namespace RapidFDM {
    namespace Aerodynamics {
        class BaseComponent {
        public:
            BaseComponent() { }

            BaseComponent(rapidjson::Value &v) {
                this->name = fast_string(_json, "name");
            }

            std::string name;
            virtual Eigen::Vector3d get_gound_velocity() {
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


            virtual Eigen::Quaterniond get_gound_attitude() {
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


            virtual Eigen::Vector3d get_ground_velocity() {
                std::abort();
            }

        };
    }
}

#endif