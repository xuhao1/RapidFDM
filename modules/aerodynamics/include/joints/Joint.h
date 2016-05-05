//
// Created by xuhao on 2016/5/2.
//

#ifndef RAPIDFDM_AERODYNAMIC_JOINT_H
#define RAPIDFDM_AERODYNAMIC_JOINT_H

#include <Eigen/Eigen>
#include <rapidjson/document.h>
#include <map>
#include <string>
#include <base_component.h>

namespace RapidFDM
{
    namespace Aerodynamics
    {
        class Node;

        class Joint : public BaseComponent
        {
        protected:
            BaseComponent *parent = nullptr;
            /*!< Parent Node for this joint*/
            BaseComponent *child = nullptr;
            /*!< Child node for this joint*/

            struct {
                //! The base transform for the joint on parent node,this transform is value middle point for the Joint
                Eigen::Affine3d parent_base_transform;
                //! This is the center point for the joint on the child node
                Eigen::Affine3d child_transform;
                //! This is the transform between base transform and the coord on child reference point
                Eigen::Affine3d relative_transform;

                //! Relative Angular Velocity
                Eigen::Vector3d relative_angular_velocity;
                //! Relative Velocity,be zero for all except free joint
                Eigen::Vector3d relative_velocity;
            } states;

            std::string type = "base_joint";
            rapidjson::Value joint_define;

        public:
            Joint(rapidjson::Value &v, std::map<std::string, Node *> nodes);

            Joint(rapidjson::Value &v, Node *_parent, Node *_child);

            Joint(Node *_parent, Node *_child) {
                assert(_child != nullptr);
            }

            //! Get the parent node for this joint
            /*!
               \return parent
             */
            BaseComponent *getParent() {
                return parent;
            }
            //! Get the child node for this joint
            /*!
               \return child
             */
            BaseComponent *getChild() {
                return child;
            }

            //! Get the transform from parent to child for this node
            /*!
               \return transform
             */
            virtual Eigen::Affine3d get_relative_transform() {
                return states.parent_base_transform * states.relative_transform * states.child_transform;
            }

            //! Get the body transform for the parent node
            /*!
               \return transform
             */
            virtual Eigen::Affine3d get_body_transform() override;

            //! Get the ground attitude for the parent node
            /*!
               \return transform
             */
            virtual Eigen::Quaterniond get_ground_attitude() override {
                return Eigen::Quaterniond(this->get_ground_transform().rotation());
            }

            virtual Eigen::Vector3d get_ground_velocity() override {
                abort();
            }

            virtual Eigen::Vector3d get_angular_velocity() override {
                abort();
            }

            virtual Eigen::Vector3d get_ground_air_speed() override {
                abort();
            }

            virtual Eigen::Affine3d get_ground_transform() override;
        };
    }
}

#endif
