//
// Created by Hao Xu on 16/7/18.
//

#ifndef RAPIDFDM_WING_BLADE_ELEMENT_H
#define RAPIDFDM_WING_BLADE_ELEMENT_H

#include <RapidFDM/aerodynamics/blade_element/base_blade_element.h>
namespace RapidFDM
{
    namespace Aerodynamics
    {
        class WingNode;
        class WingBladeElement : public BaseBladeElement {
        protected:
            //This transform shall be relative to
            Eigen::Affine3d relative_transform;
            float Mac = 0;
            bool on_taper = false;
            float deflectAngle;
            float MidChordSweep;
            float element_span_length;
            float mid_span_length;
        public:
            WingBladeElement(BaseGeometry * geo,double inner_span_percent,double outer_span_percent);

            virtual Eigen::Vector3d get_aerodynamics_torque(ComponentData data,AirState airState) const override;

            virtual Eigen::Affine3d get_relative_transform() const override ;
            virtual float get_cl(ComponentData state,AirState airState) const;

            virtual float get_cd(ComponentData state,AirState airState) const;

            virtual float get_cm(ComponentData state,AirState airState) const;

            virtual float getLift(ComponentData state, AirState airState) const override;

            virtual float getDrag(ComponentData state, AirState airState) const override;

            virtual float getSide(ComponentData state, AirState airState) const override;

            WingNode * get_wing_node() const;



        };
    }
}
#endif //RAPIDFDM_WING_BLADE_ELEMENT_H
