//
// Created by xuhao on 2016/5/2.
//

#include "RapidFDM/aerodynamics/nodes/base_node.h"
#include "RapidFDM/aerodynamics/joints/base_joint.h"
#include <RapidFDM/utils.h>

using namespace RapidFDM::Utils;

namespace RapidFDM
{
    namespace Aerodynamics
    {
        BaseNode::BaseNode(BaseJoint *_parent)
        {
            this->parent = _parent;
            inSimulate = false;
            params.mass = 0;
            params.mass_center = Eigen::Vector3d(0, 0, 0);
            name = "";
        }

        BaseNode::BaseNode(const rapidjson::Value &_json, BaseJoint *_parent) :
                BaseComponent(_json)
        {
            assert(_json.IsObject());
            init(_json, _parent);
        }


        void BaseNode::init(const rapidjson::Value &_json, BaseJoint *_parent)
        {
            this->parent = _parent;
            this->params.mass = fast_value(_json, "mass");
            this->params.mass_center = fast_vector3(_json, "mass_center");
            this->params.Inertial = fast_vector3(_json, "inertial");
            this->name = fast_string(_json, "name");
        }

        Eigen::Quaterniond BaseNode::get_ground_attitude() const
        {
            if (!inSimulate) {
                assert(parent!= nullptr);
                return this->parent->get_ground_attitude();
            }
            else
                return Eigen::Quaterniond(this->get_ground_transform().rotation());
        }

        Eigen::Affine3d BaseNode::get_body_transform() const
        {
            if (!inSimulate) {
                assert(parent!= nullptr);
                return this->parent->get_body_transform();
            }
            else
                return this->flying_states.body_transform;
        }

        Eigen::Affine3d BaseNode::get_ground_transform() const
        {
            if (!inSimulate) {
                assert(parent!= nullptr);
                return this->parent->get_ground_transform();
            }
            else
                return this->flying_states.transform;
        }

        Eigen::Vector3d BaseNode::get_angular_velocity() const
        {
            if (!inSimulate) {
                assert(parent != nullptr);
                return parent->get_angular_velocity();
            }
            else
                return this->flying_states.angular_velocity;
        }


        Eigen::Vector3d BaseNode::get_ground_velocity() const
        {
            if (!inSimulate) {
                assert(parent!= nullptr);
                return parent->get_ground_velocity();
            }
            else
                return this->flying_states.ground_velocity;
        }

        void BaseNode::init_component_data()
        {
            if (!inSimulate)
            {
                this->flying_states.body_transform = get_body_transform();
                this->flying_states.transform = get_ground_transform();
                this->flying_states.angular_velocity = Eigen::Vector3d(0,0,0);
                this->flying_states.ground_velocity = Eigen::Vector3d(0,0,0);
                this->geometry->set_flying_state(this->flying_states);
            }
        }

        std::string BaseNode::get_type_str() const
        {
            switch (node_type)
            {
                case AerodynamicsNodeType ::AerodynamicsBaseNode: {
                    return "AerodynamicsBaseNode";
                    break;
                }
                case AerodynamicsNodeType ::AerodynamicsBaseEngineNode:
                {
                    return "AerodynamicsBaseEngineNode";
                    break;
                }
                case AerodynamicsNodeType ::AerodynamicsEasyPropellerNode:
                {
                    return "AerodynamicsEasyPropellerNode";
                    break;
                }
                case AerodynamicsNodeType ::AerodynamicsAircraftNode:
                {
                    return "AerodynamicsAircraftNode";
                    break;
                }
                case AerodynamicsNodeType ::AerodynamicsWingNode:
                {
                    return "AerodynamicsWingNode";
                    break;
                }
                default:{
                    std::cerr<<"Undefined node type str " << node_type << std::endl;
                    abort();
                    return "";
                }
            }
            return "";
        }

        AerodynamicsNodeType BaseNode::get_node_type()
        {
            return node_type;
        }
    }
}
