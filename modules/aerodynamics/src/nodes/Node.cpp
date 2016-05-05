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
        Node::Node(Joint *_parent) {
           this->parent = _parent;
           inSimulate = false;
           params.mass = 0;
           params.mass_center = Eigen::Vector3d(0, 0, 0);
           name = "";
       }

        Node::Node(rapidjson::Value &_json, Joint *_parent) :
                BaseComponent(_json) {
            this->parent = _parent;
            this->params.mass = fast_value(_json, "mass");
            this->params.mass_center = fast_vector3(_json, "mass_center");
            this->params.Inertial = fast_vector3(_json, "inertial");
            this->name = fast_string(_json, "name");
        }

        Node::Node(rapidjson::Document &document, Joint *_parent) {
            rapidjson::Value &v = document;
            *this = Node(v, _parent);
        }

        Eigen::Quaterniond Node::get_ground_attitude() {
            if (!inSimulate)
                return this->parent->get_ground_attitude();
            else
                return Eigen::Quaterniond(this->flying_states.transform.rotation());
        }

        Eigen::Affine3d Node::get_body_transform() {
            if (!inSimulate)
                return this->parent->get_body_transform();
            else
                return this->flying_states.body_transform;
        }

        Eigen::Affine3d Node::get_ground_transform() {
            if (!inSimulate) {
                return this->parent->get_ground_transform();
            }
            else
                return this->flying_states.transform;
        }

        Eigen::Vector3d Node::get_angular_velocity() {
            if (!inSimulate)
                return this->parent->get_angular_velocity();
            else
                return this->flying_states.angular_velocity;
        }

        Eigen::Vector3d Node::get_air_velocity() {
            //TODO
            return Eigen::Vector3d(0, 0, 0);
        }

        Eigen::Vector3d Node::get_ground_velocity() {
            if (!inSimulate)
                return parent->get_ground_velocity();
            else
                return this->flying_states.velocity;
        }
    }
}
