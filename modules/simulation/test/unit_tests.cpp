//
// Created by Hao Xu on 16/6/12.
//

#include <RapidFDM/simulation/utils.h>

using namespace RapidFDM::Simulation::Utils;
using namespace Eigen;
void convert_tests()
{
    Eigen::Vector3d fuck(1,2,3);
    PxVec3 vec3 = vector_e2p(fuck);
    vec3 = vec3 + PxVec3(3,5,6);
    fuck = vector_p2e(vec3);
    printf("Vec convert res : %5lf %5lf %5lf\n",fuck.x(),fuck.y(),fuck.z());

    Eigen::Quaterniond quaterniond_ori(
            Eigen::AngleAxisd(0.25*M_PI, Vector3d::UnitX())
    );
    Eigen::Quaterniond quaterniond = quaterniond_ori;

    printf("Quat convert res 0 : wxyz %5lf %5lf %5lf %5lf\n",
           quaterniond.w(),
           quaterniond.x(),
           quaterniond.y(),
           quaterniond.z()
    );
    PxQuat pquat = quat_e2p(quaterniond);

    quaterniond = quaterniond * quat_p2e(pquat.getConjugate());

    printf("Quat convert res: wxyz %5lf %5lf %5lf %5lf\n",
           quaterniond.w(),
           quaterniond.x(),
           quaterniond.y(),
           quaterniond.z()
    );

    Affine3d trans;
    trans.fromPositionOrientationScale(
            fuck,
            quaterniond_ori,
            Vector3d(1,1,1)
    );
    printf("origin vec eigen %f %f %f px %f %f %f\n",
           fuck.x(),fuck.y(),fuck.z(),
           vec3.x,vec3.y,vec3.z
    );
    Vector3d eigen_transed = trans.linear() * fuck;
    printf("Eigen Trans :%5lf %5lf %5lf\n",
           eigen_transed.x(),
           eigen_transed.y(),
           eigen_transed.z()
    );

    PxTransform pxTransform = transform_e2p(trans);
    vec3 = pxTransform.rotate(vec3);
    printf("Px Trans   :%5lf %5lf %5lf\n",
           vec3.x,
           vec3.y,
           vec3.z
    );

    trans = transform_p2e(pxTransform);
    eigen_transed = trans.linear() * fuck;
    printf("Eigen Trans 2:%5lf %5lf %5lf\n",
           eigen_transed.x(),
           eigen_transed.y(),
           eigen_transed.z()
    );

}

int main()
{
    printf("Hello,world\n");
    convert_tests();
}

