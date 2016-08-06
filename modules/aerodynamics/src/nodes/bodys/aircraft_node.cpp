#include <RapidFDM/aerodynamics/nodes/bodys/aircraft_node.h>

namespace RapidFDM
{
    namespace Aerodynamics
    {
        AircraftNode::AircraftNode(const rapidjson::Value &_json) :
                BaseNode(_json)
        {
            init(_json);
        }

        void AircraftNode::init(const rapidjson::Value &_json)
        {
            assert(_json.IsObject());

            if (_json.HasMember("geometry") && _json["geometry"].IsObject()) {
                this->geometry = GeometryHelper::create_geometry_from_json(_json["geometry"]);
                this->geometry->_parent = this;
            }
            else {
                this->geometry = new BaseGeometry();
                this->geometry->_parent = this;
            }
            this->node_type = AerodynamicsNodeType ::AerodynamicsAircraftNode;
            printf("Success parse aircraft_node\n");
            printf("Name %s type: %s geometry %s\n", this->name.c_str(), this->get_type_str().c_str(),
                   geometry->get_type().c_str());

            this->rigid_mode = fast_value(_json, "rigid_mode", 0) > 0;
            if (this->rigid_mode) {
                if (_json.HasMember("total_mass") && _json.HasMember("total_inertial")) {
                    total_inertial_defined = true;
                    this->total_mass = fast_value(_json, "total_mass");
                    this->total_inertial = fast_vector3(_json, "total_inertial");
                    this->mass_center = fast_vector3(_json, "mass_center");
                }
            }
        }

        void AircraftNode::set_air_state(AirState air_state)
        {
            this->airState = air_state;
            bladeElementManager.calculate_washes(air_state,-1);
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
                BaseNode *node_ptr = pair.second.node_ptr;
                std::string state_name = pair.second.state;
                node_ptr->iter_internal_state(deltatime);
                this->internal_states[global_name] = node_ptr->get_internal_states()[state_name];
            }
        }


        AircraftNode::AircraftNode() :
                BaseNode()
        {
            this->node_type = AerodynamicsNodeType ::AerodynamicsAircraftNode;
        }

        Eigen::Affine3d AircraftNode::get_body_transform() const
        {
            //origin point is mass center
            Eigen::Affine3d transform_relative_masscenter;
            transform_relative_masscenter.fromPositionOrientationScale(
                    -mass_center,
                    Eigen::Quaterniond(1, 0, 0, 0), Eigen::Vector3d(1, 1, 1));
            return transform_relative_masscenter;
        }

        Eigen::Affine3d AircraftNode::get_ground_transform() const
        {
            if (!inSimulate)
                return Eigen::Affine3d::Identity();
            return flying_states.transform;
        }

        Eigen::Vector3d AircraftNode::get_total_force() const
        {
            assert(inited);
            Eigen::Vector3d res = Eigen::Vector3d(0,0,0);
            for (auto pair : node_list) {
                BaseNode *node_ptr = pair.second;
                res += node_ptr->get_body_transform().linear() *
                       node_ptr->get_realtime_force(node_ptr->get_component_data(),airState);
            }
            return res;
        }

        Eigen::Vector3d AircraftNode::get_total_engine_force() const
        {
            assert(inited);
            Eigen::Vector3d res = Eigen::Vector3d(0,0,0);
            for (auto pair : node_list) {
                BaseNode *node_ptr = pair.second;
                BaseEngineNode *engineNode_ptr = dynamic_cast<BaseEngineNode *>(node_ptr);
                if (engineNode_ptr != nullptr)
                    res += node_ptr->get_body_transform().linear() *
                           engineNode_ptr->get_engine_force(node_ptr->get_component_data(),airState);
            }
            return res;
        }

        Eigen::Vector3d AircraftNode::get_total_engine_torque() const
        {
            assert(inited);
            Eigen::Vector3d res = Eigen::Vector3d(0,0,0);
            for (auto pair : node_list) {
                BaseNode *node_ptr = pair.second;
                auto convert_coord = node_ptr->get_body_transform().linear();
                BaseEngineNode *engineNode_ptr = dynamic_cast<BaseEngineNode *>(node_ptr);
                if (engineNode_ptr != nullptr) {
                    Eigen::Vector3d engine_body_r = (Eigen::Vector3d) engineNode_ptr->get_body_transform().translation();
                    ComponentData data = engineNode_ptr->get_component_data();
                    res += convert_coord * engineNode_ptr->get_engine_torque(data,airState)
                           + engine_body_r.cross(convert_coord * engineNode_ptr->get_engine_force(data,airState));
                }
            }
            return res;
        }

        Eigen::Vector3d AircraftNode::get_total_aerodynamics_force() const
        {
            assert(inited);
            Eigen::Vector3d res = Eigen::Vector3d(0,0,0);
            for (auto pair : node_list) {
                BaseNode *node_ptr = pair.second;
                res += node_ptr->get_body_transform().linear() *
                       node_ptr->get_aerodynamics_force(node_ptr->get_component_data(),airState);
            }
            return res;
        }

        Eigen::Vector3d AircraftNode::get_total_torque() const
        {
            assert(inited);
            Eigen::Vector3d res = Eigen::Vector3d(0,0,0);
            for (auto pair : node_list) {
                BaseNode *node_ptr = pair.second;
                auto convert_coord = node_ptr->get_body_transform().linear();
                Eigen::Vector3d node_body_r = (Eigen::Vector3d) node_ptr->get_body_transform().translation();
                ComponentData data = node_ptr->get_component_data();
                res += convert_coord * node_ptr->get_realtime_torque(data,airState)
                       + node_body_r.cross(convert_coord * node_ptr->get_realtime_force(data,airState));
            }
            return res;
        }

        Eigen::Vector3d AircraftNode::get_total_aerodynamics_torque() const
        {
            assert(inited);
            Eigen::Vector3d res = Eigen::Vector3d(0,0,0);
            for (auto pair : node_list) {
                BaseNode *node_ptr = pair.second;
                auto convert_coord = node_ptr->get_body_transform().linear();
                ComponentData data = node_ptr->get_component_data();
                Eigen::Vector3d node_body_r = (Eigen::Vector3d) node_ptr->get_body_transform().translation();
                res += convert_coord * node_ptr->get_aerodynamics_torque(data,airState) +
                       node_body_r.cross(convert_coord * node_ptr->get_aerodynamics_force(data,airState));

            }
            return res;
        }

        //TODO:
        //Consider inertial matrix is not a diagonal matrix.
        Eigen::Vector3d AircraftNode::get_total_inertial() const
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

        double AircraftNode::get_total_mass() const
        {
            if (rigid_mode && total_inertial_defined) {
                return total_mass;
            }
            else {
//                total_mass = this->get_mass();
//                for (auto pair : node_list) {
//                    total_mass += pair.second->get_mass();
//                }
//                return total_mass;
            }
                return 0;
        }

        Eigen::Vector3d AircraftNode::get_total_mass_center() const
        {
            if (rigid_mode && total_inertial_defined) {
                return mass_center;
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
                std::map<std::string, BaseNode *> _node_list,
                std::map<std::string, BaseJoint *> _joint_list)
        {
            inited = true;
            this->node_list = _node_list;
            this->joint_list = _joint_list;
            this->node_list.erase(this->getUniqueID());
            this->init_component_data();
            for (auto pair : node_list) {
                BaseNode *node_ptr = pair.second;
                node_ptr->init_component_data();
                this->bladeElementManager.add_blade_elements(node_ptr->geometry->get_blade_elements());
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
            add_transform(source_document, get_ground_transform(), source_document);
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


        BaseNode *AircraftNode::instance()
        {
            AircraftNode *node = new AircraftNode;
            memcpy(node, this, sizeof(AircraftNode));
            node->geometry = this->geometry->instance();
            return node;
        }

        void AircraftNode::setStatefromsimulator(const ComponentData & data)
        {
            this->setSimulate(true);
            if (rigid_mode)
            {
                this->flying_states = data;
                for (auto pair : node_list)
                {
                    BaseNode * node_ptr = pair.second;
                    ComponentData node_data;
                    node_data.body_transform = node_ptr->get_body_transform();
                    node_data.angular_velocity = node_data.body_transform.linear() * data.angular_velocity;
                    node_data.ground_velocity =  data.ground_velocity +
                            data.angular_velocity.cross((Eigen::Vector3d)node_data.body_transform.translation());
                    node_data.transform = data.transform * node_data.body_transform;
                    node_ptr->setStatefromsimulator(node_data);

                }
            }
            else
            {
                this->flying_states = data;
            }

        }

        void AircraftNode::calculate_washes(AirState airState,double deltatime)
        {
            this->bladeElementManager.calculate_washes(airState,deltatime);
        }

        Eigen::Vector3d AircraftNode::get_angular_velocity() const
        {
            return this->flying_states.angular_velocity;
        }


    }
}
