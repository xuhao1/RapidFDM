//
// Created by xuhao on 2016/5/2.
//

#ifndef RAPIDFDM_NODE_H
#define RAPIDFDM_NODE_H

#include <Eigen/Eigen>
#include <vector>

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
                Eigen::Vector3d velocity; /*!< Velocity of center */
                Eigen::Vector3d angular_velocity; /*!< Angular Velocity of center */
                Eigen::Quaterniond attitude_relative; /*!< attitude relative to the root */
                Eigen::Quaterniond attitude_ground; /*< absolute attitude,using for calcuate gravity torque if gravity center is not 0,0,0*/
            } states;

            struct {
                double mass; /*!< Mass of this node, in kilogram*/
                Eigen::Vector3d gravity_center;/*!< Gravity center to the center*/
            } params;

            Joint * parent = nullptr; /*!< Parent joint of this, be null if standalone */

        public:
            Node(Joint * _parent = nullptr)
            {
                states.velocity = Eigen::Vector3d(0,0,0);
                states.angular_velocity = Eigen::Vector3d(0,0,0);
                states.attitude_relative = Eigen::Quaterniond(1,0,0,0);
                states.attitude_ground = Eigen::Quaterniond(1,0,0,0);
                params.mass = 0;
                params.gravity_center = Eigen::Vector3d(0,0,0);
            }

            //! A Calucate total force of this node
            /*!
              \return The calucated realtime force
            */
            virtual Eigen::Vector3d get_realtime_force() = 0;

            //! A Calucate total torque of this node
            /*!
              \return The calucated realtime torque
            */
            virtual Eigen::Vector3d get_realtime_torque() = 0;


            // protocols

            Eigen::Quaterniond get_gound_attitude()
            {
                return states.attitude_ground;
            }

            Eigen::Quaterniond get_relative_attitude()
            {
                return states.attitude_relative;
            }

            void set_mass(double _mass,Eigen::Vector3d gravity_center = Eigen::Vector3d(0,0,0))
            {
                params.mass = _mass;
                params.gravity_center = gravity_center;
            }

        };
    }
}


#endif //RAPIDFDM_NODE_H
