#include <RapidFDM/aerodynamics/base_component.h>

namespace RapidFDM{
    namespace Aerodynamics{
        Eigen::Vector3d BaseAerodynamicsComponent::get_aerodynamics_force(ComponentData data, AirState airState) const
        {
                //TODO:
            //fix these codes
            Eigen::Vector3d force_tmp = Eigen::Vector3d
                    (getDrag(data, airState), getSide(data, airState),
                     getLift(data, airState));

            double alpha = data.get_angle_of_attack(airState);
            double beta =  - data.get_sideslip(airState);

            Eigen::Matrix3d body2wind;

            body2wind << cos(alpha) * cos(beta), sin(beta), sin(alpha) * cos(beta)
                    , -cos(alpha) * sin(beta), cos(beta), -sin(alpha) * sin(beta)
                    , -sin(alpha), 0, cos(alpha);

            force_tmp = body2wind.inverse() * force_tmp;
            return force_tmp;

        }
    }
}