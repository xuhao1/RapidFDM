//
// Created by xuhao on 2016/5/3.
//

#ifndef RAPIDFDM_FLYINGDATA_H
#define RAPIDFDM_FLYINGDATA_H

#include <Eigen/Eigen>
#include <utils.h>

using namespace RapidFDM::Utils;

namespace RapidFDM {
    namespace Aerodynamics {
        struct ComponentData {
            Eigen::Affine3d transform;
            /*!< Transform from simulator */
            Eigen::Affine3d body_transform;
            /*!< Body transform from simulator */
            Eigen::Vector3d angular_velocity;
            /*!< Angular velocity from simulator on component transform*/
            Eigen::Vector3d velocity;

            /*!<Velocity from simulator on the ground*/
            rapidjson::Value encode2json(rapidjson::Document &d) {
                rapidjson::Value v;
                add_transform(v, transform, d, "transform");
                add_transform(v, body_transform, d, "body_transform");
                add_vector(v, angular_velocity, d, "angular_velocity");
                add_vector(v, velocity, d, "velocity");
                return v;
            }
        };

    }
}
#endif //RAPIDFDM_FLYINGDATA_H
