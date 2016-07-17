//
// Created by Hao Xu on 16/7/18.
//

#ifndef RAPIDFDM_BASE_BLADE_ELEMENT_H
#define RAPIDFDM_BASE_BLADE_ELEMENT_H

#include <RapidFDM/aerodynamics/geometrys/Geometrys.h>
#include <RapidFDM/aerodynamics/base_component.h>

namespace RapidFDM
{
    namespace Aerodynamics
    {
        class BaseBladeElement : public BaseMoveableComponent , public BaseAerodynamicsComponent {
        protected:
            BaseGeometry * geometry = nullptr;
            Eigen::Vector3d relative_pos;
        public:

            BaseBladeElement(BaseGeometry * geo,Eigen::Vector3d _relative_pos = Eigen::Vector3d(0,0,0));

            virtual Eigen::Vector3d get_position_relative_geometry() const;

            Eigen::Affine3d get_relative_transform() const;

            virtual Eigen::Vector3d get_ground_velocity();

            virtual Eigen::Vector3d get_angular_velocity();

            virtual Eigen::Affine3d get_ground_transform();

            virtual Eigen::Quaterniond get_ground_attitude();

            virtual Eigen::Affine3d get_body_transform();

            ComponentData make_component_data_from_geometry(ComponentData data) const;
            virtual float getLift(ComponentData state, AirState airState)
            {
                return 0;
            }

            virtual float getDrag(ComponentData state, AirState airState)
            {
                return 0;
            }

            virtual float getSide(ComponentData state, AirState airState)
            {
                return 0;
            }

        };
    }
}

#endif //RAPIDFDM_BASE_BLADE_ELEMENT_H
