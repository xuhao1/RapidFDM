//
// Created by xuhao on 2016/5/4.
//


#ifndef RAPIDFDM_BASE_GEOMETRY_H
#define RAPIDFDM_BASE_GEOMETRY_H

#include <RapidFDM/aerodynamics/flying_data_defines.h>
#include <iostream>
#include <stdio.h>
#include <RapidFDM/aerodynamics/base_component.h>

namespace RapidFDM
{
    namespace Aerodynamics
    {

        class BaseGeometry : public BaseComponent
        {
        protected:
            float Aero = 0;
            /*!< aera for aerodynamics calucation */
            std::string type = "base";
        public:
            BaseGeometry() :
                    BaseComponent()
            {

            }

            BaseGeometry(const rapidjson::Value &v) :
                    BaseComponent(v)
            {

            }

            std::string get_type()
            {
                return type;
            };

            //TODO:
            //Write codes
            virtual float getLift(ComponentData state, AirState airState)
            {
//                std::cerr << "Code not wrote" << std::endl;
//                abort();
                return 0;
            }

            virtual float getDrag(ComponentData state, AirState airState)
            {
//                std::cerr << "Code not wrote" << std::endl;
//                abort();
                return 0;
            }

            virtual float getSide(ComponentData state, AirState airState)
            {
//                std::cerr << "Code not wrote" << std::endl;
//                abort();
                return 0;
            }

            virtual Eigen::Vector3d get_aerodynamics_center()
            {
                return Eigen::Vector3d(0, 0, 0);
            }

            virtual Eigen::Vector3d getTorque(ComponentData state, AirState airState)
            {
                return get_aerodynamics_center().cross(getForce(state, airState));
            }

            Eigen::Vector3d getForce(ComponentData state, AirState airState);

            virtual BaseGeometry *instance()
            {
//                BaseComponent * baseComponent
                abort();
            }

            virtual void brief()
            {
                printf("Geometry %s\n", type.c_str());
            }
        };
    }

}

#endif //RAPIDFDM_BASE_GEOMETRY_H
