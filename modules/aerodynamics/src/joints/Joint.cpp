//
// Created by xuhao on 2016/5/2.
//

#include <RapidFDM/aerodynamics/joints/Joint.h>
#include <RapidFDM/utils.h>
#include <RapidFDM/aerodynamics/joints/Joint.h>
#include <RapidFDM/aerodynamics/nodes/Node.h>

using namespace RapidFDM::Utils;

namespace RapidFDM
{
    namespace Aerodynamics
    {
        //!Construct Joint from a series of json
        /*!
            \param value
                - base_position: [x y z] required
                - base_rotation : [x y z] / [w x y z] required
                - child_reference_point [x y z] optional [0 0 0]
         */
        void Joint::init(const rapidjson::Value &v, Node *_parent, Node *_child)
        {

            assert(_child != nullptr);
            assert(_parent != nullptr);
            this->child = _child;
            _parent->add_joint(this);

            Eigen::Vector3d base_pos = fast_vector3(v, "base_position");
            Eigen::Quaterniond base_rot = fast_attitude(v, "base_rotation");
            Eigen::Vector3d child_point = fast_vector3(v, "child_reference_point");
            Eigen::Vector3d base_scale(1, 1, 1);
            states.parent_base_transform.fromPositionOrientationScale(base_pos, base_rot, base_scale);
            states.child_transform.fromPositionOrientationScale(-child_point, Eigen::Quaterniond(1, 0, 0, 0),
                                                                base_scale);
            states.relative_transform.fromPositionOrientationScale(
                    Eigen::Vector3d(0, 0, 0),
                    Eigen::Quaterniond(1, 0, 0, 0),
                    Eigen::Vector3d(1, 1, 1)
            );
            this->parent = _parent;
            child->parent = this;
        }

        Joint::Joint(const rapidjson::Value &v, Node *_parent, Node *_child) :
                BaseComponent(v)
        {
            init(v, _parent, _child);
        }

        Joint::Joint(const rapidjson::Value &v, std::map<std::string, Node *> nodes) :
                BaseComponent(v)
        {
            std::string parent_id;
            std::string child_id;

            assert (v.HasMember("parent") && v["parent"].IsString());
            parent_id = v["parent"].GetString();
            assert (v.HasMember("child") && v["child"].IsString());
            child_id = v["child"].GetString();

            assert(nodes.find(parent_id)!=nodes.end());
            assert(nodes.find(child_id)!=nodes.end());

            parent = nodes[parent_id];
            child = nodes[child_id];
            init(v, parent, child);

        }

        Eigen::Affine3d Joint::get_ground_transform()
        {
            assert(parent != nullptr);
            return parent->get_ground_transform() * get_relative_transform();
        }

        Eigen::Affine3d Joint::get_body_transform()
        {
            assert(parent != nullptr);
            return parent->get_body_transform() * get_relative_transform();
        }

        Joint::Joint(Node *_parent, Node *_child)
        {
            assert(_child != nullptr);
            if (_parent != nullptr) {
                _parent->add_joint(this);
            }
        }

//! Get the parent node for this joint
    }
}