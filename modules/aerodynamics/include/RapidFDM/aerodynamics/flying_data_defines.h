//
// Created by xuhao on 2016/5/3.
//

#ifndef RAPIDFDM_FLYINGDATA_H
#define RAPIDFDM_FLYINGDATA_H

#include <Eigen/Eigen>
#include <RapidFDM/utils.h>

using namespace RapidFDM::Utils;

namespace RapidFDM
{
    namespace Aerodynamics
    {
        struct AirState
        {
            //! air speed
            Eigen::Vector3d ground_air_speed = Eigen::Vector3d(0, 0, 0);
            //! air density
            float rho = 1.29;

            AirState()
            {
            }
        };

        struct ComponentData
        {
            /*!< Transform from simulator */
            Eigen::Affine3d transform = Eigen::Affine3d::Identity();

            /*!< Body transform from simulator */
            Eigen::Affine3d body_transform = Eigen::Affine3d::Identity();

            /*!< Angular velocity from simulator on component transform should be in body cooridinate*/
            Eigen::Vector3d angular_velocity = Eigen::Vector3d(0,0,0);

            /*!< Ground velocity for this component */
            Eigen::Vector3d ground_velocity = Eigen::Vector3d(0,0,0);


            Eigen::Vector3d get_relative_airspeed(AirState airState) const
            {
                Eigen::Vector3d speed_in_air = - ground_velocity + airState.ground_air_speed;
                return - (transform.linear().inverse() * speed_in_air);
            }

            double get_q_bar(AirState airState) const
            {
                //Q bar will get airspeed on x & y axis!!!!
                return airState.rho * this->get_front_airspeed(airState) * this->get_front_airspeed(airState) / 2;
//                return airState.rho * this->get_airspeed_mag(airState) * this->get_airspeed_mag(airState) / 2;
            }

            double get_angle_of_attack(AirState airState) const
            {
                if (get_airspeed_mag(airState) < 0.1)
                {
                    return 0;
                }

                auto relative_airspeed = get_relative_airspeed(airState);
                double aoa =  atan2(- relative_airspeed.z(), - relative_airspeed.x());
                return float_constrain(aoa,-M_PI/2,M_PI/2);
            }
            double get_front_airspeed(AirState airState) const
            {
                return - sqrt(pow(get_relative_airspeed(airState).x(),2) +
                        pow(get_relative_airspeed(airState).z(),2));
            }
            double get_airspeed_mag(AirState airState) const
            {
                return get_relative_airspeed(airState).norm();
            }

            double get_sideslip(AirState airState) const
            {
                if (get_airspeed_mag(airState) < 0.1)
                {
                    return 0;
                }

                auto relative_airspeed = get_relative_airspeed(airState);
                double beta =  atan2(relative_airspeed.y(), - relative_airspeed.x());
                return beta;
            }

            ComponentData()
            {

            }

            /*!<Velocity from simulator on the ground*/
            rapidjson::Value encode2json(rapidjson::Document &d)
            {
                rapidjson::Value v;
                add_transform(v, transform, d, "transform");
                add_transform(v, body_transform, d, "body_transform");
                add_vector(v, angular_velocity, d, "angular_velocity");
                add_vector(v, ground_velocity, d, "velocity");
                return v;
            }
        };


    }
}
#endif //RAPIDFDM_FLYINGDATA_H
