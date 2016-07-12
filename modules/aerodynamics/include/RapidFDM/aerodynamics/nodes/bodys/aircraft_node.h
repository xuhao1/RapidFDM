#ifndef __RAPIDFDM_AERODYNAMIC_NODE_H__
#define __RAPIDFDM_AERODYNAMIC_NODE_H__

#include <RapidFDM/aerodynamics/nodes/Node.h>
#include <RapidFDM/aerodynamics/FlyingData.h>
#include <RapidFDM/aerodynamics/nodes/engines/base_engine.h>
#include <rapidjson/document.h>
#include <RapidFDM/aerodynamics/geometrys/geometry_helper.h>
#include <stdio.h>
#include <RapidFDM/aerodynamics/nodes/engines/base_engine.h>

namespace RapidFDM
{
    namespace Aerodynamics
    {
        struct node_control_axis
        {
            Node *node_ptr = nullptr;
            std::string axis;

            node_control_axis()
            {
            }

            node_control_axis(Node *_node_ptr, std::string _axis)
            {
                node_ptr = _node_ptr;
                axis = _axis;
            }
        };

        struct node_internal_state
        {
            Node *node_ptr = nullptr;
            std::string state;

            node_internal_state()
            {
            }

            node_internal_state(Node *_node_ptr, std::string _state)
            {
                node_ptr = _node_ptr;
                state = _state;
            }
        };

        //! This node stand for the base of a aircraft
        class AircraftNode : public Node
        {
        protected:
            bool inited = false;
            std::map<std::string, Node *> node_list;
            std::map<std::string, Joint *> joint_list;

            std::map<std::string, node_control_axis> control_axis_mapper;
            std::map<std::string, node_internal_state> internal_state_mapper;

            //If enable static mode,all component in this aircraft will be static to the aircraft
            bool rigid_mode = false;
            //If total inertial defined, we will read intertial in json file,
            //This will only make sense when static_mode is true
            bool total_inertial_defined = false;

            Eigen::Vector3d total_inertial = Eigen::Vector3d(0, 0, 0);
            double total_mass = 0;
            Eigen::Vector3d mass_center_offset = Eigen::Vector3d(0, 0, 0);
            void init(const rapidjson::Value &_json);

        public:
            AircraftNode(const rapidjson::Value &_json);


            virtual void set_air_state(AirState air_state) override;

            virtual void set_control_value(std::string name, double v) override;

            virtual void iter_internal_state(double deltatime) override;

            AircraftNode();

            virtual Eigen::Affine3d get_body_transform() override;

            virtual Eigen::Vector3d get_total_force();

            virtual Eigen::Vector3d get_total_engine_force();

            virtual Eigen::Vector3d get_total_engine_torque();

            virtual Eigen::Vector3d get_total_aerodynamics_force();

            virtual Eigen::Vector3d get_total_torque();

            virtual Eigen::Vector3d get_total_aerodynamics_torque();

            //TODO:
            //Consider inertial matrix is not a diagonal matrix.
            virtual Eigen::Vector3d get_total_inertial();

            virtual double get_total_mass();

            virtual Eigen::Vector3d get_total_mass_center();

            void init_after_construct(
                    std::map<std::string, Node *> _node_list,
                    std::map<std::string, Joint *> _joint_list);


            virtual Node *instance() override;

            virtual const rapidjson::Value & getJsonDefine() override;
            virtual const rapidjson::Document * getComponentsDefine();

        };
    }
}


#endif