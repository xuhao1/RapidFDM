//
// Created by xuhao on 2016/5/3.
//

#ifndef RAPIDFDM_FLYINGDATA_H
#define RAPIDFDM_FLYINGDATA_H

#include <Eigen/Eigen>
#include <utils.h>
#include <rapidfdmlcm/airstate.hpp>

using namespace RapidFDM::Utils;

namespace RapidFDM {
    namespace Aerodynamics {
        struct AirState {
            //! air speed
            Eigen::Vector3d ground_air_speed = Eigen::Vector3d(0, 0, 0);
            //! air density
            float rho = 1.29;
            AirState(rapidfdmlcm::airstate airstate)
            {
                this->rho = airstate.rho;
                this->ground_air_speed.x() = airstate.wind_speed[0];
                this->ground_air_speed.y() = airstate.wind_speed[1];
                this->ground_air_speed.z() = airstate.wind_speed[2];
            }
            rapidfdmlcm::airstate &operator=(const AirState& rhs)
            {
                rapidfdmlcm::airstate airstate;
                airstate.rho = rhs.rho;
                airstate.wind_speed[0] = ground_air_speed.x();
                airstate.wind_speed[1] = ground_air_speed.y();
                airstate.wind_speed[2] = ground_air_speed.z();
            }
            AirState()
            {}
        };

        struct ComponentData {
            /*!< Transform from simulator */
            Eigen::Affine3d transform;

            /*!< Body transform from simulator */
            Eigen::Affine3d body_transform;

            /*!< Angular velocity from simulator on component transform*/
            Eigen::Vector3d angular_velocity;

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
