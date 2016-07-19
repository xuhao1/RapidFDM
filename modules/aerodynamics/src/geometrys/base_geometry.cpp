//
// Created by Hao Xu on 16/7/14.
//
#include <RapidFDM/aerodynamics/geometrys/base_geometry.h>
#include <RapidFDM/aerodynamics/nodes/base_node.h>


namespace RapidFDM
{
    namespace Aerodynamics
    {
        Eigen::Affine3d BaseGeometry::get_body_transform() const
        {
            return _parent->get_body_transform();
        }

        Eigen::Affine3d BaseGeometry::get_ground_transform() const
        {
            return _parent->get_ground_transform();
        }

        Eigen::Quaterniond BaseGeometry::get_ground_attitude() const
        {
            return _parent->get_ground_attitude();
        }
        Eigen::Vector3d BaseGeometry::get_ground_velocity() const
        {
            return _parent->get_ground_velocity();
        }
        Eigen::Vector3d BaseGeometry::get_angular_velocity() const
        {
            return _parent->get_angular_velocity();
        }
    }
}
