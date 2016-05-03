//
// Created by xuhao on 2016/5/2.
//

#include <airdynamics/include/nodes/Node.h>
#include "nodes/Node.h"
#include "joints/Joint.h"
#include <utils.h>

using namespace RapidFDM::Utils;

namespace RapidFDM
{
    namespace Aerodynamics
    {
       Node::Node(Joint * _parent) {
           this->parent = _parent;
       }

        Node::Node(rapidjson::Value &_json, Joint *_parent) :
                Node(_parent) {
            this->params.mass = fast_value(_json, "mass");
            this->params.mass_center = fast_vector3(_json, "mass_center");
            if (_json.HasMember("name") && _json["name"].IsString()) {
                this->name = _json["name"].GetString();
            }
        }

        Eigen::Quaterniond Node::get_gound_attitude() {
            return this->parent->get_ground_attitude();
        }

        Eigen::Affine3d Node::get_body_transform() {
            return this->parent->get_body_transform();
        }

        Eigen::Affine3d Node::get_gound_transform() {
            return this->parent->get_ground_transform();
        }
    }
}
