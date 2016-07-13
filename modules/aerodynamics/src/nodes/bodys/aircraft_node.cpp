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

        void AircraftNode::init(const rapidjson::Value &_json)
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
            if (this->rigid_mode) {
                if (_json.HasMember("total_mass") && _json.HasMember("total_inertial")) {
                    total_inertial_defined = true;
                    this->total_mass = fast_value(_json, "total_mass");
                    this->total_inertial = fast_vector3(_json, "total_inertial");
                    this->mass_center_offset = fast_vector3(_json, "mass_center");
                }
            }
        }

        void AircraftNode::set_air_state(AirState air_state)
        {
            this->flying_states.airState = air_state;
            for (auto pair : node_list) {
                Node *node_ptr = pair.second;
                node_ptr->set_air_state(air_state);
            }
        }

        int AircraftNode::set_control_value(std::string name, double v)
        {
            if (this->control_axis_mapper.find(name) != this->control_axis_mapper.end()) {
                this->control_axis[name] = v;
                node_control_axis node_control = this->control_axis_mapper[name];
                node_control.node_ptr->set_control_value(
                        node_control.axis,
                        v
                );
                return 0;
            }
            return -1;
        }

        int AircraftNode::set_internal_state(std::string name, double v)
        {
            if (this->internal_state_mapper.find(name) != this->internal_state_mapper.end()) {
                this->internal_states[name] = v;
                node_internal_state node_internal_state1 = this->internal_state_mapper[name];
                node_internal_state1.node_ptr->set_internal_state(
                        node_internal_state1.state,
                        v
                );
                return 0;
            }
            return -1;
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
                res += node_ptr->get_body_transform().linear() * node_ptr->get_realtime_force(node_ptr->get_component_data());
//                std::cout << node_ptr->getName() << " force : " <<
//                node_ptr->get_realtime_force(node_ptr->get_component_data()) << std::endl;
            }
//            std::cout << "Total force:" << res << std::endl;
            return res;
        }

        Eigen::Vector3d AircraftNode::get_total_engine_force()
        {
            assert(inited);
            Eigen::Vector3d res;
            for (auto pair : node_list) {
                Node *node_ptr = pair.second;
                BaseEngineNode *engineNode_ptr = dynamic_cast<BaseEngineNode *>(node_ptr);
                if (engineNode_ptr != nullptr)
                    res += node_ptr->get_body_transform().linear() * engineNode_ptr->get_engine_force(node_ptr->get_component_data());
            }
            return res;
        }

        Eigen::Vector3d AircraftNode::get_total_engine_torque()
        {
            assert(inited);
            Eigen::Vector3d res;
            for (auto pair : node_list) {
                Node *node_ptr = pair.second;
                BaseEngineNode *engineNode_ptr = dynamic_cast<BaseEngineNode *>(node_ptr);
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
                res += node_ptr->get_body_transform().linear() * node_ptr->get_airdynamics_force(node_ptr->get_component_data());
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
            if (rigid_mode && total_inertial_defined) {
                return total_inertial;
            }
            else {
                //TODO:
                //Count total inertial
                abort();
                return Eigen::Vector3d(0, 0, 0);
            }
        }

        double AircraftNode::get_total_mass()
        {
            if (rigid_mode && total_inertial_defined) {
                return total_mass;
            }
            else {
                total_mass = this->get_mass();
                for (auto pair : node_list) {
                    total_mass += pair.second->get_mass();
                }
                return total_mass;
            }
        }

        Eigen::Vector3d AircraftNode::get_total_mass_center()
        {
            if (rigid_mode && total_inertial_defined) {
                return mass_center_offset;
            }
            else {
                for (auto pair : node_list) {
//                    mass_center_offset += pair.second->get_mass() * pair;
                    //TODO:
                    //calcuate mass center offset
                    abort();
                }
                return Eigen::Vector3d(0, 0, 0);
            }
        }

        void AircraftNode::init_after_construct(
                std::map<std::string, Node *> _node_list,
                std::map<std::string, Joint *> _joint_list)
        {
            inited = true;
            this->node_list = _node_list;
            this->joint_list = _joint_list;
            this->node_list.erase(this->getUniqueID());
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

        const rapidjson::Value &AircraftNode::getJsonDefine()
        {
            add_transform(source_document, get_body_transform(), source_document);
            rapidjson::Value namev(rapidjson::kStringType);
            namev.SetString("nodes", source_document.GetAllocator());
            rapidjson::Value geometrys(rapidjson::kObjectType);
            geometrys.CopyFrom(*getComponentsDefine(), source_document.GetAllocator());
            source_document.AddMember(namev, geometrys, source_document.GetAllocator());
            return source_document;
        }


        const rapidjson::Document *AircraftNode::getComponentsDefine()
        {
            rapidjson::Document *d = new rapidjson::Document;
            d->SetObject();

            for (auto pair:node_list) {
                rapidjson::Value namev(rapidjson::kStringType);
                namev.SetString(pair.first.c_str(), d->GetAllocator());
                rapidjson::Value geo_def(rapidjson::kObjectType);
                geo_def.CopyFrom(pair.second->getJsonDefine(), d->GetAllocator());
                d->AddMember(namev, geo_def, d->GetAllocator());
            }
            return d;

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
