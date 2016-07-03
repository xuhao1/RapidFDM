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

        Node::Node(const rapidjson::Value &_json, rapidjson::Document &document, Joint *_parent) :
                BaseComponent(_json)
        {
            assert(_json.IsObject());
            init(_json, document, _parent);
        }

        Node::Node(rapidjson::Document &document, Joint *_parent)
        {
            rapidjson::Value &v = document;
            init(v, document, _parent);
        }

        void Node::init(rapidjson::Value &_json, rapidjson::Document &document, Joint *_parent)
        {
            this->parent = _parent;
            this->params.mass = fast_value(_json, "mass");
            this->params.mass_center = fast_vector3(_json, "mass_center");
            this->params.Inertial = fast_vector3(_json, "inertial");
            this->name = fast_string(_json, "name");

            this->describer.CopyFrom(_json, document.GetAllocator());
            this->source_document = document;
        }

        Eigen::Quaterniond Node::get_ground_attitude()
        {
            if (!inSimulate)
                return this->parent->get_ground_attitude();
            else
                return Eigen::Quaterniond(this->flying_states.transform.rotation());
        }

        Eigen::Affine3d Node::get_body_transform()
        {
            if (!inSimulate)
                return this->parent->get_body_transform();
            else
                return this->flying_states.body_transform;
        }

        Eigen::Affine3d Node::get_ground_transform()
        {
            if (!inSimulate) {
                return this->parent->get_ground_transform();
            }
            else
                return this->flying_states.transform;
        }

        Eigen::Vector3d Node::get_angular_velocity()
        {
            if (!inSimulate)
                return this->parent->get_angular_velocity();
            else
                return this->flying_states.angular_velocity;
        }


        Eigen::Vector3d Node::get_ground_velocity()
        {
            if (!inSimulate)
                return parent->get_ground_velocity();
            else
                return this->flying_states.velocity;
        }
    }
}
