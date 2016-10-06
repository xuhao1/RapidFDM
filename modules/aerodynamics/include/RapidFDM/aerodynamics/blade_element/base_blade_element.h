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

        public:

            BaseBladeElement(BaseGeometry * geo);

            virtual Eigen::Affine3d get_relative_transform() const;

            virtual Eigen::Vector3d get_ground_velocity() const override ;

            virtual Eigen::Vector3d get_angular_velocity() const override ;

            virtual Eigen::Affine3d get_ground_transform() const override ;

            virtual Eigen::Quaterniond get_ground_attitude() const override ;

            virtual Eigen::Affine3d get_body_transform() const override ;

            ComponentData make_component_data_from_geometry(ComponentData data) const;
            ComponentData update_component_data_from_geometry(ComponentData data);
            ComponentData get_component_data_from_geometry();
            ComponentData get_seted_component_data();
            virtual float getLift(ComponentData state, AirState airState) const
            {
                return 0;
            }

            virtual float getDrag(ComponentData state, AirState airState) const
            {
                return 0;
            }

            virtual float getSide(ComponentData state, AirState airState) const
            {
                return 0;
            }


        };
    }
}

#endif //RAPIDFDM_BASE_BLADE_ELEMENT_H
