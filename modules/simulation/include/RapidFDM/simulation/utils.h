//
// Created by Hao Xu on 16/6/12.
//

#ifndef RAPIDFDM_UTILS_H
#define RAPIDFDM_UTILS_H

#ifndef NDEBUG
#define NDEBUG
#endif

#include <Eigen/Eigen>
#include <foundation/PxTransform.h>
#include <foundation/PxQuat.h>

using namespace physx;
using namespace Eigen;

namespace RapidFDM
{
    namespace Simulation
    {
        namespace Utils
        {
            inline PxVec3 vector_e2p(const Eigen::Vector3d &vec)
            {
                return PxVec3(vec.x(), vec.y(), vec.z());
            }

            inline Eigen::Vector3d vector_p2e(const PxVec3 &vec)
            {
                return Eigen::Vector3d(vec.x, vec.y, vec.z);
            }

            inline PxQuat quat_e2p(const Eigen::Quaterniond &quat)
            {
                return PxQuat(quat.x(), quat.y(), quat.z(), quat.w());
            }

            inline Eigen::Quaterniond quat_p2e(const PxQuat &quat)
            {
                Eigen::Quaterniond Quat;
                Quat.w() = quat.w;
                Quat.x() = quat.x;
                Quat.y() = quat.y;
                Quat.z() = quat.z;
                return Quat;
            }

            inline PxTransform transform_e2p(const Eigen::Affine3d &trans)
            {
                return PxTransform(
                        vector_e2p((Eigen::Vector3d) trans.translation()),
                        quat_e2p((Eigen::Quaterniond) trans.rotation())
                );
            }

            inline Eigen::Affine3d transform_p2e(const PxTransform &trans)
            {
                Eigen::Affine3d res;
                res.fromPositionOrientationScale(
                        vector_p2e(trans.p),
                        quat_p2e(trans.q),
                        Vector3d(1, 1, 1)
                );
                return res;
            }
        }

    }
}
#endif //RAPIDFDM_UTILS_H
