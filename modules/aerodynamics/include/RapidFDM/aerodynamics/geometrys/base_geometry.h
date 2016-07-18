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
