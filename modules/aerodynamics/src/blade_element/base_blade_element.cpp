//
// Created by Hao Xu on 16/7/18.
//

#include <RapidFDM/aerodynamics/blade_element/base_blade_element.h>

namespace RapidFDM
{
    namespace Aerodynamics
    {
        BaseBladeElement::BaseBladeElement(BaseGeometry *geo) :
                geometry(geo)
        {
            assert(geometry != nullptr);
        }

        Eigen::Affine3d BaseBladeElement::get_body_transform() const
        {
            return geometry->get_body_transform() * get_relative_transform();
        }

        Eigen::Affine3d BaseBladeElement::get_ground_transform() const
        {
            return geometry->get_ground_transform() * get_relative_transform();
        }

        Eigen::Vector3d BaseBladeElement::get_ground_velocity() const
        {
            return geometry->get_ground_velocity() +
                    geometry->get_angular_velocity().cross(
                            (Eigen::Vector3d) get_relative_transform().translation());
        }

        Eigen::Vector3d BaseBladeElement::get_angular_velocity() const
        {
            return geometry->get_angular_velocity();
        }

        Eigen::Quaterniond BaseBladeElement::get_ground_attitude() const
        {
            return geometry->get_ground_attitude();
        }

        ComponentData BaseBladeElement::make_component_data_from_geometry(ComponentData data) const
        {
            data.body_transform = data.body_transform * this->get_relative_transform();
            data.transform = data.transform * this->get_relative_transform();
            data.ground_velocity = data.ground_velocity +
                                   geometry->get_angular_velocity().cross(
                                           (Eigen::Vector3d) get_relative_transform().translation());
            data.angular_velocity = get_relative_transform().linear() * data.angular_velocity;
            return data;
        }
    }
}