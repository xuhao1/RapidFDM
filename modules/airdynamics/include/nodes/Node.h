//
// Created by xuhao on 2016/5/2.
//

#ifndef RAPIDFDM_NODE_H
#define RAPIDFDM_NODE_H

#include <Eigen/Eigen>
#include <vector>
#include <airdynamics/include/joints/Joint.h>
#include <rapidjson/rapidjson.h>
#include <rapidjson/document.h>

namespace RapidFDM
{

    namespace Aerodynamics
    {
        class Joint;

        class Node
        {

        protected:
            std::vector<Joint*> linked_joints; /*!< List of linked joints*/


            struct {
                Eigen::Vector3d velocity;
                /*!< Relative velocity of center */
                Eigen::Vector3d angular_velocity; /*!<Relative angular Velocity of center */
            } states;

            struct {
                double mass; /*!< Mass of this node, in kilogram*/
                Eigen::Vector3d mass_center;/*!< Mass center to the center*/
            } params;

            Joint * parent = nullptr; /*!< Parent joint of this, be null if standalone */

        public:
            std::string name;
            Node(Joint * _parent = nullptr)
            {
                states.velocity = Eigen::Vector3d(0,0,0);
                states.angular_velocity = Eigen::Vector3d(0,0,0);
                params.mass = 0;
                params.mass_center = Eigen::Vector3d(0, 0, 0);
                name = "";
            }

            //! Calucate total force of this node
            /*!
              \return The calucated realtime force
            */
            virtual Eigen::Vector3d get_realtime_force() = 0;

            //! A Calucate total torque of this node
            /*!
              \return The calucated realtime torque
            */
            virtual Eigen::Vector3d get_realtime_torque() = 0;

            Node(rapidjson::Value &_json, Joint *_parent = nullptr);
            // protocols

            Eigen::Quaterniond get_gound_attitude();

            Eigen::Affine3d get_body_transform();

            Eigen::Affine3d get_gound_transform();

            void set_mass(double _mass, Eigen::Vector3d mass_center = Eigen::Vector3d(0, 0, 0))
            {
                params.mass = _mass;
                params.mass_center = mass_center;
            }

        };
    }
}


#endif //RAPIDFDM_NODE_H
