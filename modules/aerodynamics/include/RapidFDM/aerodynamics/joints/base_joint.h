//
// Created by xuhao on 2016/5/2.
//

#ifndef RAPIDFDM_AERODYNAMIC_JOINT_H
#define RAPIDFDM_AERODYNAMIC_JOINT_H

#include <Eigen/Eigen>
#include <rapidjson/document.h>
#include <map>
#include <string>
#include <RapidFDM/aerodynamics/base_component.h>

enum AerodynamicsJointType
{
    AerodyanmicsBaseJoint,
    AerodynamicsFixedJoint
};

namespace RapidFDM
{
    namespace Aerodynamics
    {
        class BaseNode;

        class BaseJoint : public BaseComponent
        {
        protected:
            BaseNode *parent = nullptr;
            /*!< Parent BaseNode for this joint*/
            BaseNode *child = nullptr;
            /*!< Child node for this joint*/

            struct
            {
                //! The base transform for the joint on parent node,this transform is value middle point for the BaseJoint
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
            AerodynamicsJointType joint_type = AerodynamicsJointType::AerodyanmicsBaseJoint;
            rapidjson::Value joint_define;
            void init(const rapidjson::Value &v, BaseNode *_parent, BaseNode *_child);

        public:
            BaseJoint(const rapidjson::Value &v, std::map<std::string, BaseNode *> nodes);

            BaseJoint(const rapidjson::Value &v, BaseNode *_parent, BaseNode *_child);

            BaseJoint(BaseNode *_parent, BaseNode *_child);


            void init(rapidjson::Value &v, BaseNode *_parent, BaseNode *_child);
            //! Get the parent node for this joint
            /*!
               \return parent
             */
            BaseNode *getParent()
            {
                return parent;
            }
            //! Get the child node for this joint
            /*!
               \return child
             */
            BaseNode *getChild()
            {
                return child;
            }

            //! Get the type for this joint
            virtual AerodynamicsJointType getType()
            {
                return joint_type;
            }

            //! Get the transform from parent to child for this node
            /*!
               \return transform
             */
            virtual Eigen::Affine3d get_relative_transform() const
            {
                return states.parent_base_transform * states.relative_transform * states.child_transform;
            }

            //! Get the body transform for the parent node
            /*!
               \return transform
             */
            virtual Eigen::Affine3d get_body_transform() const override;

            //! Get the ground attitude for the parent node
            /*!
               \return transform
             */
            virtual Eigen::Quaterniond get_ground_attitude() const override
            {
                return Eigen::Quaterniond(this->get_ground_transform().rotation());
            }

            virtual Eigen::Vector3d get_ground_velocity() const override
            {
                abort();
            }

            virtual Eigen::Vector3d get_angular_velocity() const override
            {
                abort();
            }


            virtual Eigen::Affine3d get_ground_transform() const override;

            virtual void brief() override
            {
                printf("This is base joint\n");
            }

            virtual Eigen::Affine3d get_base_transform()
            {
                return states.parent_base_transform;
            }

            //! Return the joint in child frame
            virtual Eigen::Affine3d get_child_transform()
            {
                return states.child_transform.inverse();
            }
        };
    }
}

#endif
