//
// Created by xuhao on 2016/5/2.
//

#ifndef __RAPIDFDM_AERODYNAMICS_FREEJOINT_H__
#define __RAPIDFDM_AERODYNAMICS_FREEJOINT_H__

#include <joints/Joint.h>

namespace RapidFDM {
    namespace Aerodynamics {
        class FreeJoint : public Joint {
            FreeJoint(Node *_child) :
                    Joint(nullptr, _child) {

            }
        };
    }
};
#endif

