//
// Created by xuhao on 2016/5/5.
//

#ifndef RAPIDFDM_AIRDYNAMICS_FIXED_JOINT_H
#define RAPIDFDM_AIRDYNAMICS_FIXED_JOINT_H

#include <joints/Joint.h>
#include <nodes/Node.h>

namespace RapidFDM {
    namespace Aerodynamics {
        class FixedJoint : public Joint {
        public:
            FixedJoint(rapidjson::Value &v, std::map<std::string, Node *> nodes) :
                    Joint(v, nodes) {
                this->type = "fixed";
            }

            FixedJoint(rapidjson::Value &v, Node *_parent, Node *_child) :
                    Joint(v, _parent, _child) {
                this->type = "fixed";
            }

            FixedJoint(Node *_parent, Node *_child) :
                    Joint(_parent, _child) {
                this->type = "fixed";
            }
        };
    }
}

#endif //RAPIDFDM_FIXED_JOINT_H
