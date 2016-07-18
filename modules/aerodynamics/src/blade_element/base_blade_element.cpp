//
// Created by Hao Xu on 16/7/18.
//

#include <RapidFDM/aerodynamics/blade_element/base_blade_element.h>

namespace RapidFDM{
    namespace Aerodynamics {
        BaseBladeElement::BaseBladeElement(BaseGeometry * geo,Eigen::Vector3d _relative_pos):
                geometry(geo),relative_pos(_relative_pos)
        {
            assert(geometry!= nullptr);
        }
        Eigen::Vector3d BaseBladeElement::get_position_relative_geometry() const
        {
            return relative_pos;
        }
        Eigen::Affine3d BaseBladeElement::get_relative_transform() const
        {
            Eigen::Affine3d trans;
            trans.fromPositionOrientationScale(
                    get_position_relative_geometry(),
                    Eigen::Quaterniond(1,0,0,0),
                    Eigen::Vector3d(1,1,1)
            );
            return trans;
        }
        Eigen::Affine3d BaseBladeElement::get_body_transform() const
        {
//            return geometry->get_body_transform() * get_relative_transform();
            assert(true);
        }
        Eigen::Affine3d BaseBladeElement::get_ground_transform() const
        {
//            return geometry->get_ground_transform() * get_relative_transform();
            assert(true);
        }
        Eigen::Vector3d BaseBladeElement::get_ground_velocity() const
        {
//            return geometry->get_ground_velocity() +
//                    geometry->get_angular_velocity().cross(
//                            get_position_relative_geometry());
            assert(true);
        }
        Eigen::Vector3d BaseBladeElement::get_angular_velocity() const
        {
//            return geometry->get_angular_velocity();
            assert(true);
        }
        Eigen::Quaterniond BaseBladeElement::get_ground_attitude() const
        {
//            return geometry->get_ground_attitude();
            assert(true);
        }

        ComponentData BaseBladeElement::make_component_data_from_geometry(ComponentData data) const
        {
            data.body_transform = data.body_transform * this->get_relative_transform();
            data.transform = data.transform * this->get_relative_transform();
            data.ground_velocity = data.ground_velocity +
                    geometry->get_angular_velocity().cross(
                            get_position_relative_geometry());
            data.angular_velocity = data.angular_velocity;
            return data;
        }
    }
}