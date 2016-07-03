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

        public:
            AircraftNode(const rapidjson::Value &_json, rapidjson::Document &document) :
                    Node(_json, document)
            {
                init(_json);
            }

            AircraftNode(rapidjson::Document &document) :
                    Node(document)
            {
                init(document);
            }

            void init(const rapidjson::Value &_json)
            {
                assert(_json.IsObject());

                if (_json.HasMember("geometry") && _json["geometry"].IsObject()) {
                    this->geometry = GeometryHelper::create_geometry_from_json(_json["geometry"]);
                }
                else {
                    this->geometry = new BaseGeometry();
                }
                this->type_str = "aircraft_node";
                printf("Success parse aircraft_node\n");
                printf("Name %s type: %s geometry %s\n", this->name.c_str(), this->type_str.c_str(),
                       geometry->get_type().c_str());

                this->rigid_mode = fast_value(_json, "rigid_mode", 0) > 0;

            }

            virtual void set_air_state(AirState air_state) override
            {
                this->flying_states.airState = air_state;
                for (auto pair : node_list) {
                    Node *node_ptr = pair.second;
                    node_ptr->set_air_state(air_state);
                }
            }

            virtual void set_control_value(std::string name, double v) override
            {
                this->control_axis[name] = v;
                if (this->control_axis_mapper.find(name) != this->control_axis_mapper.end()) {
                    node_control_axis node_control = this->control_axis_mapper[name];
                    node_control.node_ptr->set_control_value(
                            node_control.axis,
                            v
                    );
                }
            }

            virtual void iter_internal_state(double deltatime) override
            {
                for (auto pair:internal_state_mapper) {
                    std::string global_name = pair.first;
                    Node *node_ptr = pair.second.node_ptr;
                    std::string state_name = pair.second.state;
                    node_ptr->iter_internal_state(deltatime);
                    this->internal_states[global_name] = node_ptr->get_internal_states()[state_name];
                }
            }

            AircraftNode() :
                    Node()
            {
                this->type_str = "aircraft_node";
            }

            virtual Eigen::Affine3d get_body_transform() override
            {
                return Eigen::Affine3d::Identity();
            }

            virtual Eigen::Vector3d get_total_force()
            {
                assert(inited);
                Eigen::Vector3d res;
                for (auto pair : node_list) {
                    Node *node_ptr = pair.second;
                    res += node_ptr->get_realtime_force(node_ptr->get_component_data());
                }
                return res;
            }

            virtual Eigen::Vector3d get_engine_force()
            {
                assert(inited);
                Eigen::Vector3d res;
                for (auto pair : node_list) {
                    Node *node_ptr = pair.second;
                    BaseEngineNode *engineNode_ptr = static_cast<BaseEngineNode *>(node_ptr);
                    if (engineNode_ptr != nullptr)
                        res += engineNode_ptr->get_engine_force(node_ptr->get_component_data());
                }
                return res;
            }

            virtual Eigen::Vector3d get_engine_torque()
            {
                assert(inited);
                Eigen::Vector3d res;
                for (auto pair : node_list) {
                    Node *node_ptr = pair.second;
                    BaseEngineNode *engineNode_ptr = static_cast<BaseEngineNode *>(node_ptr);
                    if (engineNode_ptr != nullptr) {
                        Eigen::Vector3d engine_body_r = (Eigen::Vector3d) engineNode_ptr->get_body_transform().translation();
                        ComponentData data = engineNode_ptr->get_component_data();
                        res += engineNode_ptr->get_engine_torque(data)
                               + engine_body_r.cross(engineNode_ptr->get_engine_force(data));
                    }
                }
                return res;
            }

            virtual Eigen::Vector3d get_total_aerodynamics_force()
            {
                assert(inited);
                Eigen::Vector3d res;
                for (auto pair : node_list) {
                    Node *node_ptr = pair.second;
                    res += node_ptr->get_airdynamics_force(node_ptr->get_component_data());
                }
                return res;
            }

            virtual Eigen::Vector3d get_total_torque()
            {
                assert(inited);
                Eigen::Vector3d res;
                for (auto pair : node_list) {
                    Node *node_ptr = pair.second;
                    Eigen::Vector3d node_body_r = (Eigen::Vector3d) node_ptr->get_body_transform().translation();
                    ComponentData data = node_ptr->get_component_data();
                    res += node_ptr->get_realtime_torque(data)
                           + node_body_r.cross(node_ptr->get_realtime_force(data));
                }
                return res;
            }

            virtual Eigen::Vector3d get_total_aerodynamics_torque()
            {
                assert(inited);
                Eigen::Vector3d res;
                for (auto pair : node_list) {
                    Node *node_ptr = pair.second;
                    Eigen::Vector3d node_body_r = (Eigen::Vector3d) node_ptr->get_body_transform().translation();
                    ComponentData data = node_ptr->get_component_data();
                    res += node_ptr->get_airdynamics_torque(data) +
                           node_body_r.cross(node_ptr->get_airdynamics_force(data));

                }
                return res;
            }

            //TODO:
            //Consider inertial matrix is not a diagonal matrix.
            virtual Eigen::Vector3d get_total_inertial()
            {

            }

            virtual Eigen::Vector3d get_total_mass()
            {

            }

            void init_after_construct(
                    std::map<std::string, Node *> _node_list,
                    std::map<std::string, Joint *> _joint_list)
            {
                inited = true;
                this->node_list = _node_list;
                this->joint_list = _joint_list;
                this->node_list.erase(this->name);
                for (auto pair : node_list) {
                    Node *node_ptr = pair.second;
                    for (auto internal_pair :node_ptr->get_internal_states()) {
                        std::string state_name = internal_pair.first;
                        this->internal_states[node_ptr->getUniqueID() + "/" + state_name] = internal_pair.second;
                        this->internal_state_mapper[node_ptr->getUniqueID() + "/" + state_name] =
                                node_internal_state(node_ptr, state_name);
                    }

                    for (auto axis_pair :node_ptr->get_control_axis()) {
                        std::string axis_name = axis_pair.first;
                        this->control_axis[node_ptr->getUniqueID() + "/" + axis_name] = axis_pair.second;
                        this->control_axis_mapper[node_ptr->getUniqueID() + "/" + axis_name] =
                                node_control_axis(node_ptr, axis_name);
                    }
                }

            }


            virtual Node *instance() override
            {
                AircraftNode *node = new AircraftNode;
                memcpy(node, this, sizeof(AircraftNode));
                node->geometry = this->geometry->instance();
                return node;
            }
        };
    }
}


#endif