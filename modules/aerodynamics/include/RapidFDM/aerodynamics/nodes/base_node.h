//
// Created by xuhao on 2016/5/2.
//

#ifndef RAPIDFDM_NODE_H
#define RAPIDFDM_NODE_H

#include <Eigen/Eigen>
#include <vector>
#include <RapidFDM/aerodynamics/joints/base_joint.h>
#include <rapidjson/rapidjson.h>
#include <rapidjson/document.h>
#include <RapidFDM/aerodynamics/flying_data_defines.h>
#include <RapidFDM/aerodynamics/base_component.h>
#include <RapidFDM/aerodynamics/geometrys/base_geometry.h>
#include <stdio.h>


namespace RapidFDM
{

    namespace Aerodynamics
    {
        class BaseJoint;

        enum AerodynamicsNodeType
        {
            AerodynamicsBaseNode,
            AerodynamicsAircraftNode,
            AerodynamicsBaseEngineNode,
            AerodynamicsEasyPropellerNode,
            AerodynamicsWingNode
        };

        class BaseNode : public BaseComponent
        {

        protected:
            std::vector<BaseJoint *> linked_joints;
            /*!< List of linked joints*/

            struct
            {
                double mass;
                /*!< Mass of this node, in kilogram*/
                Eigen::Vector3d mass_center;
                /*!< Mass center to the center*/
                Eigen::Vector3d Inertial;/*!< Inertial for this*/
            } params;

            bool inSimulate = false;

            AerodynamicsNodeType node_type = AerodynamicsNodeType ::AerodynamicsBaseNode;

            void init(const rapidjson::Value &_json, BaseJoint *_parent);
        public:

            std::string get_type_str() const;

            AerodynamicsNodeType get_node_type();

            BaseJoint *parent = nullptr;

            /*!< Parent joint of this, be null if standalone */

            BaseGeometry *geometry = nullptr;

            BaseNode(BaseJoint *_parent = nullptr);

            BaseNode(const rapidjson::Value &_json, BaseJoint *_parent = nullptr);

            //! Calucate total force of this node
            /*!
              \return The calucated realtime force
            */
            virtual Eigen::Vector3d get_realtime_force(ComponentData data,AirState airState) const
            {
                return get_aerodynamics_force(data,airState);
            };

            //! A Calucate total torque of this node,
            /*!
              \return The calucated realtime torque
            */
            virtual Eigen::Vector3d get_realtime_torque(ComponentData data,AirState airState) const
            {
                return get_aerodynamics_torque(data,airState);
            }

            //! Calucate aerodynamics force of this node
            /*!
              \return The calucated realtime aerodynamics force
            */
            virtual Eigen::Vector3d get_aerodynamics_force(ComponentData data,AirState airState) const override
            {
                if (this->geometry != nullptr) {
                    return this->geometry->get_aerodynamics_force(data, airState);
                }
                return Eigen::Vector3d(0, 0, 0);
            }

            //! Calucate aerodynamics torque of this node
            /*!
              \return The calucated realtime aerodynamics torque
            */
            virtual Eigen::Vector3d get_aerodynamics_torque(ComponentData data,AirState airState) const override
            {
                if (this->geometry != nullptr) {
                    return this->geometry->get_aerodynamics_torque(data, airState);
                }
                return Eigen::Vector3d(0, 0, 0);
            }


            virtual double get_mass()
            {
                return params.mass;
            }

            virtual Eigen::Vector3d get_mass_center()
            {
                return params.mass_center;
            }

            virtual Eigen::Vector3d get_inertial()
            {
                return params.Inertial;
            }

            //TODO:
            //Give a realitics bounding box
            virtual Eigen::Vector3d get_bounding_box()
            {
                return Eigen::Vector3d(0.1, 0.1, 0.1);
            }


            //!Air velocity relative to node in local transform,shall consider velocity from angular speed at center point
            //
            virtual Eigen::Vector3d get_air_velocity(ComponentData data,AirState airState) const {
                return data.get_relative_airspeed(airState);
            }

            virtual void init_component_data();


            //Overrides
            virtual Eigen::Quaterniond get_ground_attitude() const override;

            //Body transform is relative to mass center, not the inititalized center
            virtual Eigen::Affine3d get_body_transform() const override;

            virtual Eigen::Affine3d get_ground_transform() const override;

            virtual Eigen::Vector3d get_ground_velocity() const override;

            virtual Eigen::Vector3d get_angular_velocity() const override;

            virtual void set_mass(double _mass, Eigen::Vector3d mass_center = Eigen::Vector3d(0, 0, 0))
            {
                params.mass = _mass;
                params.mass_center = mass_center;
            }

            virtual void setSimulate(bool EnableSimulator)
            {
                this->inSimulate = EnableSimulator;
                for (BaseJoint *joint : linked_joints) {
                    joint->getChild()->setSimulate(EnableSimulator);
                }
            }

            virtual void setStatefromsimulator(const ComponentData & data)
            {
                this->flying_states = data;
                this->geometry->set_flying_state(data);
            };

            virtual void brief() override
            {
                printf("name : %s \n", name.c_str());
                printf("type : %s \n", get_type_str().c_str());
                printf("mass : %5f \n", params.mass);
                printf("Inertial %5f %5f %5f \n", params.Inertial.x(), params.Inertial.y(), params.Inertial.z());
                if (this->geometry != nullptr) {
                    this->geometry->brief();
                }
                printf("\n\n");
            }

            std::vector<BaseJoint *> get_linked_joints()
            {
                return linked_joints;
            }

            void add_joint(BaseJoint *joint)
            {
                this->linked_joints.push_back(joint);
            }

            ComponentData get_component_data()
            {
                return flying_states;
            }

            void init(rapidjson::Value &_json, BaseJoint *_parent);

            virtual BaseNode *instance()
            {
//                BaseNode * node = new BaseNode();
//                memcpy(node,this, sizeof(BaseNode));
//                node->geometry = this->geometry->instance();
                printf("Type %s not wrote instance", get_type_str().c_str());
                abort();
                return nullptr;
            }
            virtual const rapidjson::Value & getJsonDefine() override
            {
                add_transform(source_document,get_ground_transform(),source_document);
                return source_document;
            }

        };
    }
}


#endif //RAPIDFDM_NODE_H
