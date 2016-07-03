#include <RapidFDM/aerodynamics/nodes/bodys/aircraft_node.h>

namespace RapidFDM
{
    namespace Aerodynamics
    {
        AircraftNode::AircraftNode(const rapidjson::Value &_json) :
                Node(_json)
        {
            init(_json);
        }

        void AircraftNode::init(rapidjson::Value &_json, Joint *_parent)
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

        void AircraftNode::set_air_state(AirState air_state)
        {
            this->flying_states.airState = air_state;
            for (auto pair : node_list) {
                Node *node_ptr = pair.second;
                node_ptr->set_air_state(air_state);
            }
        }

        void AircraftNode::set_control_value(std::string name, double v)
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

        void AircraftNode::iter_internal_state(double deltatime)
        {
            for (auto pair:internal_state_mapper) {
                std::string global_name = pair.first;
                Node *node_ptr = pair.second.node_ptr;
                std::string state_name = pair.second.state;
                node_ptr->iter_internal_state(deltatime);
                this->internal_states[global_name] = node_ptr->get_internal_states()[state_name];
            }
        }


        AircraftNode::AircraftNode() :
                Node()
        {
            this->type_str = "aircraft_node";
        }

        Eigen::Affine3d AircraftNode::get_body_transform()
        {
            return Eigen::Affine3d::Identity();
        }

        Eigen::Vector3d AircraftNode::get_total_force()
        {
            assert(inited);
            Eigen::Vector3d res;
            for (auto pair : node_list) {
                Node *node_ptr = pair.second;
                res += node_ptr->get_realtime_force(node_ptr->get_component_data());
            }
            return res;
        }

        Eigen::Vector3d AircraftNode::get_engine_force()
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

        Eigen::Vector3d AircraftNode::get_engine_torque()
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

        Eigen::Vector3d AircraftNode::get_total_aerodynamics_force()
        {
            assert(inited);
            Eigen::Vector3d res;
            for (auto pair : node_list) {
                Node *node_ptr = pair.second;
                res += node_ptr->get_airdynamics_force(node_ptr->get_component_data());
            }
            return res;
        }

        Eigen::Vector3d AircraftNode::get_total_torque()
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

        Eigen::Vector3d AircraftNode::get_total_aerodynamics_torque()
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
        Eigen::Vector3d AircraftNode::get_total_inertial()
        {

        }

        Eigen::Vector3d AircraftNode::get_total_mass()
        {

        }

        void AircraftNode::init_after_construct(
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


        Node *AircraftNode::instance()
        {
            AircraftNode *node = new AircraftNode;
            memcpy(node, this, sizeof(AircraftNode));
            node->geometry = this->geometry->instance();
            return node;
        }

    }
}
