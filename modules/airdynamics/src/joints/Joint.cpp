//
// Created by xuhao on 2016/5/2.
//

#include <joints/Joint.h>
#include <utils.h>
#include <airdynamics/include/joints/Joint.h>
#include <nodes/Node.h>

using namespace RapidFDM::Utils;

namespace RapidFDM {
    namespace Aerodynamics {
        //!Construct Joint from a series of json
        /*!
            \param value
                - base_position: [x y z] required
                - base_rotation : [x y z] / [w x y z] required
                - child_reference_point [x y z] optional [0 0 0]
         */

        Joint::Joint(rapidjson::Value &v, Node *_parent, Node *_child) :
                Joint(_parent, _child) {
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
        }

        Joint::Joint(rapidjson::Value &v, std::map<std::string, Node *> nodes) {
            Node *parent = nullptr;
            Node *child = nullptr;
            std::string parent_name;
            std::string child_name;

            if (v.HasMember("parent") && v["parent"].IsString()) {
                parent_name = v["parent"].GetString();
            }
            if (v.HasMember("child") && v["child"].IsString()) {
                child_name = v["child"].GetString();
            }

            parent = nodes[parent_name];
            child = nodes[child_name];
            *this = Joint(v, parent, child);

        }

        Eigen::Affine3d Joint::get_ground_transform() {
            if (parent != nullptr)
                return parent->get_ground_transform() * get_relative_transform();
            return get_relative_transform();
        }

        Eigen::Affine3d Joint::get_body_transform() {
            if (parent != nullptr)
                return parent->get_body_transform() * get_relative_transform();
            return get_relative_transform();
        }

    }
}