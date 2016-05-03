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
#include <FlyingData.h>

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
                double mass; /*!< Mass of this node, in kilogram*/
                Eigen::Vector3d mass_center;/*!< Mass center to the center*/
                Eigen::Vector3d Inertial;/*!< Inertial for this*/
            } params;

            ComponentData flying_states;

            Joint * parent = nullptr; /*!< Parent joint of this, be null if standalone */

            bool inSimulate = false;
        public:
            std::string name;

            Node(Joint *_parent = nullptr);

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

            Eigen::Affine3d get_ground_transform();

            Eigen::Vector3d get_ground_velocity();

            //!Air velocity relative to node in local transform,shall consider velocity from angular speed at center point
            //
            Eigen::Vector3d get_air_velocity();

            Eigen::Vector3d get_angular_velocity();

            void set_mass(double _mass, Eigen::Vector3d mass_center = Eigen::Vector3d(0, 0, 0))
            {
                params.mass = _mass;
                params.mass_center = mass_center;
            }

            void setSimulate(bool EnableSimulator) {
                this->inSimulate = EnableSimulator;
            }

            void setSetfromsimulator(ComponentData flyingstates) {
                this->flying_states = flyingstates;
            };

        };
    }
}


#endif //RAPIDFDM_NODE_H
