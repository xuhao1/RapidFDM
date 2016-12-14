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

        class BaseBladeElement;
        class BaseNode;
        class BaseGeometry : public BaseComponent
        {
        protected:
            float Aero = 0;
            /*!< aera for aerodynamics calucation */
            std::string type = "base";
            std::vector<BaseBladeElement *> blades;
        public:
            BaseNode * _parent;
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

            virtual void brief() override
            {
                printf("Geometry %s\n", type.c_str());
            }

            std::vector<BaseBladeElement *> get_blade_elements()
            {
                return this->blades;
            }

            virtual Eigen::Vector3d get_ground_velocity() const override ;

            virtual Eigen::Vector3d get_angular_velocity() const override;

            virtual Eigen::Affine3d get_ground_transform() const override ;

            virtual Eigen::Quaterniond get_ground_attitude() const override;

            virtual Eigen::Affine3d get_body_transform() const override ;

            virtual ComponentData get_flying_state()
            {
                return this->flying_states;
            }
            virtual void set_flying_state(const ComponentData & data)
            {
                this->flying_states = data;
            }

        };
    }

}

#endif //RAPIDFDM_BASE_GEOMETRY_H
