//
// Created by xuhao on 2016/5/5.
//

#ifndef RAPIDFDM_AIRDYNAMICS_FIXED_JOINT_H
#define RAPIDFDM_AIRDYNAMICS_FIXED_JOINT_H

#include <RapidFDM/aerodynamics/joints/base_joint.h>
#include <RapidFDM/aerodynamics/nodes/base_node.h>

namespace RapidFDM {
    namespace Aerodynamics {
        class FixedJoint : public BaseJoint {
        public:
            FixedJoint(const rapidjson::Value &v, std::map<std::string, BaseNode *> nodes) :
                    BaseJoint(v, nodes) {
                this->type = "fixed";
                this->joint_type = AerodynamicsJointType ::AerodynamicsFixedJoint;
            }

            FixedJoint(const rapidjson::Value &v, BaseNode *_parent, BaseNode *_child) :
                    BaseJoint(v, _parent, _child) {
                this->type = "fixed";
                this->joint_type = AerodynamicsJointType ::AerodynamicsFixedJoint;
            }

            FixedJoint(BaseNode *_parent, BaseNode *_child) :
                    BaseJoint(_parent, _child) {
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
