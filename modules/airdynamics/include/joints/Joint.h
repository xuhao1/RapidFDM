//
// Created by xuhao on 2016/5/2.
//

#ifndef RAPIDFDM_AERODYNAMIC_JOINT_H
#define RAPIDFDM_AERODYNAMIC_JOINT_H

#include <Eigen/Eigen>
#include <rapidjson/document.h>
#include <map>
#include <string>

namespace RapidFDM
{
    namespace Aerodynamics
    {
        class Node;
        class Joint
        {
        protected:
            Node *parent = nullptr;
            /*!< Parent Node for this joint*/
            Node *child = nullptr;
            /*!< Child node for this joint*/

            struct {
                //! The base transform for the joint on parent node,this transform is value middle point for the Joint
                Eigen::Affine3d parent_base_transform;
                //! This is the center point for the joint on the child node
                Eigen::Affine3d child_transform;
                //! This is the transform between base transform and the coord on child reference point
                Eigen::Affine3d relative_transform;
            } states;

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
            Node *getParent() {
                return parent;
            }
            //! Get the child node for this joint
            /*!
               \return child
             */
            Node *getChild() {
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
            virtual Eigen::Affine3d get_body_transform();

            //! Get the ground attitude for the parent node
            /*!
               \return transform
             */
            virtual Eigen::Quaterniond get_ground_attitude() {
                return this->get_gound_transform().rotation();
            }

            virtual Eigen::Affine3d get_gound_transform();
        };
    }
}

#endif
