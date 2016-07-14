//
// Created by xuhao on 2016/5/2.
//

#include "RapidFDM/aerodynamics/nodes/Node.h"
#include "RapidFDM/aerodynamics/joints/Joint.h"
#include <RapidFDM/utils.h>

using namespace RapidFDM::Utils;

namespace RapidFDM
{
    namespace Aerodynamics
    {
        Node::Node(Joint *_parent)
        {
            this->parent = _parent;
            inSimulate = false;
            params.mass = 0;
            params.mass_center = Eigen::Vector3d(0, 0, 0);
            name = "";
        }

        Node::Node(const rapidjson::Value &_json, Joint *_parent) :
                BaseComponent(_json)
        {
            assert(_json.IsObject());
            init(_json, _parent);
        }


        void Node::init(const rapidjson::Value &_json, Joint *_parent)
        {
            this->parent = _parent;
            this->params.mass = fast_value(_json, "mass");
            this->params.mass_center = fast_vector3(_json, "mass_center");
            this->params.Inertial = fast_vector3(_json, "inertial");
            this->name = fast_string(_json, "name");
        }

        Eigen::Quaterniond Node::get_ground_attitude()
        {
            if (!inSimulate) {
                assert(parent!= nullptr);
                return this->parent->get_ground_attitude();
            }
            else
                return Eigen::Quaterniond(this->get_ground_transform().rotation());
        }

        Eigen::Affine3d Node::get_body_transform()
        {
            if (!inSimulate) {
                assert(parent!= nullptr);
                return this->parent->get_body_transform();
            }
            else
                return this->flying_states.body_transform;
        }

        Eigen::Affine3d Node::get_ground_transform()
        {
            if (!inSimulate) {
                assert(parent!= nullptr);
                return this->parent->get_ground_transform();
            }
            else
                return this->flying_states.transform;
        }

        Eigen::Vector3d Node::get_angular_velocity()
        {
            if (!inSimulate) {
                assert(parent != nullptr);
                return this->parent->get_angular_velocity();
            }
            else
                return this->flying_states.angular_velocity;
        }


        Eigen::Vector3d Node::get_ground_velocity()
        {
            if (!inSimulate) {
                assert(parent!= nullptr);
                return parent->get_ground_velocity();
            }
            else
                return this->flying_states.velocity;
        }

        void Node::init_component_data()
        {
            if (!inSimulate)
            {
                this->flying_states.body_transform = get_body_transform();
                this->flying_states.transform = get_ground_transform();
                this->flying_states.angular_velocity = Eigen::Vector3d(0,0,0);
                this->flying_states.velocity = Eigen::Vector3d(0,0,0);
            }
        }
    }
}
