#ifndef __RAPIDFDM_AERODYNAMIC_NODE_H__
#define __RAPIDFDM_AERODYNAMIC_NODE_H__

#include <RapidFDM/aerodynamics/nodes/base_node.h>
#include <RapidFDM/aerodynamics/flying_data_defines.h>
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
            BaseNode *node_ptr = nullptr;
            std::string axis;

            node_control_axis()
            {
            }

            node_control_axis(BaseNode *_node_ptr, std::string _axis)
            {
                node_ptr = _node_ptr;
                axis = _axis;
            }
        };

        struct node_internal_state
        {
            BaseNode *node_ptr = nullptr;
            std::string state;

            node_internal_state()
            {
            }

            node_internal_state(BaseNode *_node_ptr, std::string _state)
            {
                node_ptr = _node_ptr;
                state = _state;
            }
        };

        //! This node stand for the base of a aircraft
        class AircraftNode : public BaseNode
        {
        protected:
            bool inited = false;
            std::map<std::string, BaseNode *> node_list;
            std::map<std::string, BaseJoint *> joint_list;

            std::map<std::string, node_control_axis> control_axis_mapper;
            std::map<std::string, node_internal_state> internal_state_mapper;

            //If enable static mode,all component in this aircraft will be static to the aircraft
            bool rigid_mode = false;
            //If total inertial defined, we will read intertial in json file,
            //This will only make sense when static_mode is true
            bool total_inertial_defined = false;

            Eigen::Vector3d total_inertial = Eigen::Vector3d(0, 0, 0);
            double total_mass = 0;
            Eigen::Vector3d mass_center = Eigen::Vector3d(0, 0, 0);
            void init(const rapidjson::Value &_json);

        public:
            AircraftNode(const rapidjson::Value &_json);


            virtual void set_air_state(AirState air_state) override;

            virtual int set_control_value(std::string name, double v) override;

            virtual int set_internal_state(std::string name,double v) override;

            virtual void iter_internal_state(double deltatime) override;

            AircraftNode();

            virtual Eigen::Affine3d get_body_transform() const override;

            virtual Eigen::Affine3d get_ground_transform() const override;

            virtual Eigen::Vector3d get_total_force() const;

            virtual Eigen::Vector3d get_total_engine_force() const;

            virtual Eigen::Vector3d get_total_engine_torque() const;

            virtual Eigen::Vector3d get_total_aerodynamics_force() const;

            virtual Eigen::Vector3d get_total_torque() const;

            virtual Eigen::Vector3d get_total_aerodynamics_torque() const;

            //TODO:
            //Consider inertial matrix is not a diagonal matrix.
            virtual Eigen::Vector3d get_total_inertial() const;

            virtual double get_total_mass() const;

            virtual Eigen::Vector3d get_total_mass_center() const;

            void init_after_construct(
                    std::map<std::string, BaseNode *> _node_list,
                    std::map<std::string, BaseJoint *> _joint_list);


            virtual BaseNode *instance() override;

            virtual const rapidjson::Value & getJsonDefine() override;
            virtual const rapidjson::Document * getComponentsDefine();
            virtual void setStatefromsimulator(const ComponentData & data) override;
            bool is_rigid()
            {
                return rigid_mode;
            }

        };
    }
}


#endif