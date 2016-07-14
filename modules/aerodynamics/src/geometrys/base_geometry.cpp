//
// Created by Hao Xu on 16/7/14.
//
#include <RapidFDM/aerodynamics/geometrys/base_geometry.h>


namespace RapidFDM
{
    namespace Aerodynamics
    {
        Eigen::Vector3d BaseGeometry::getForce(ComponentData state, AirState airState)
        {
            //TODO:
            //fix these codes
            Eigen::Vector3d force_tmp = Eigen::Vector3d(getDrag(state, airState), getSide(state, airState),
                                                        getLift(state, airState));

            double alpha = state.get_angle_of_attack(airState);
            double beta = state.get_sideslip(airState);

            Eigen::Matrix3d body2wind;

            body2wind << cos(alpha) * cos(beta), sin(beta), sin(alpha) * cos(beta)
                    , -cos(alpha) * sin(beta), cos(beta), -sin(alpha) * sin(beta)
                    , -sin(alpha), 0, cos(alpha);

            printf("force tmp %5.4f %5.4f %5.4f \n",force_tmp.x(),force_tmp.y(),force_tmp.z());
            force_tmp = body2wind.inverse() * force_tmp;
            printf("force     %5.4f %5.4f %5.4f \n",force_tmp.x(),force_tmp.y(),force_tmp.z());
            return force_tmp;


        }

    }
}
