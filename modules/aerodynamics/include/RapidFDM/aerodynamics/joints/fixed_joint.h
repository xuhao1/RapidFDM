//
// Created by xuhao on 2016/5/5.
//

#ifndef RAPIDFDM_AIRDYNAMICS_FIXED_JOINT_H
#define RAPIDFDM_AIRDYNAMICS_FIXED_JOINT_H

#include <RapidFDM/aerodynamics/joints/Joint.h>
#include <RapidFDM/aerodynamics/nodes/Node.h>

namespace RapidFDM {
    namespace Aerodynamics {
        class FixedJoint : public Joint {
        public:
            FixedJoint(rapidjson::Value &v, std::map<std::string, Node *> nodes) :
                    Joint(v, nodes) {
                this->type = "fixed";
                this->joint_type = AerodynamicsJointType ::AerodynamicsFixedJoint;
            }

            FixedJoint(rapidjson::Value &v, Node *_parent, Node *_child) :
                    Joint(v, _parent, _child) {
                this->type = "fixed";
                this->joint_type = AerodynamicsJointType ::AerodynamicsFixedJoint;
            }

            FixedJoint(Node *_parent, Node *_child) :
                    Joint(_parent, _child) {
                this->type = "fixed";
                this->joint_type = AerodynamicsJointType ::AerodynamicsFixedJoint;
            }

            virtual void brief() {
                printf("This is Fixed joint\n");
            }
        };
    }
}

#endif //RAPIDFDM_FIXED_JOINT_H
