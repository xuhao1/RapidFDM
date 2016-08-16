//
// Created by Hao Xu on 16/6/11.
//

#include <RapidFDM/aerodynamics/aerodynamics_configurer.h>

using namespace RapidFDM::Utils;
namespace RapidFDM
{
    namespace Aerodynamics
    {
        aerodynamics_configurer::aerodynamics_configurer(std::string root_path)
        {
            this->root_path = root_path;
            init_query_functions();
        }

        std::vector<std::string> aerodynamics_configurer::chroot_folder(std::string path)
        {
            this->root_path = path;
            return list_model();
        }

        std::vector<std::string> aerodynamics_configurer::list_model()
        {
            return get_filename_list(root_path);
        }

        const rapidjson::Value &aerodynamics_configurer::load_model(std::string name)
        {
            if (parser1 != nullptr) {
                delete parser1;
                parser1 = nullptr;
            }

            parser1 = new parser(root_path + "/" + name);

            this->aircraftNode = parser1->get_aircraft_node();

            return this->aircraftNode->getJsonDefine();
        }

        void aerodynamics_configurer::save_model(std::string name)
        {
        }

        void aerodynamics_configurer::update_model(const rapidjson::Value &v)
        {

        }

        rapidjson::Document *aerodynamics_configurer::query_model(const rapidjson::Value &v)
        {
            std::string query_code = fast_string(v, "opcode");
            if (aircraftNode != nullptr && query_functions.find(query_code) != query_functions.end()) {
                return query_functions[query_code](v);
            }
            else {
                rapidjson::Document *d = new rapidjson::Document;
                d->SetObject();
                d->AddMember("opcode", "response_query", d->GetAllocator());
                rapidjson::Value str(rapidjson::kStringType);
                str.SetString(query_code.c_str(), d->GetAllocator());
                d->AddMember("type", str, d->GetAllocator());
                d->AddMember("data", "no object loaded", d->GetAllocator());
                return d;
            }
        }

        rapidjson::Document *aerodynamics_configurer::query_configurer(const rapidjson::Value &v)
        {
            std::string query_code = fast_string(v, "opcode");

            if (query_configure_functions.find(query_code) != query_configure_functions.end()) {
                return query_configure_functions[query_code](v);
            }
            else {
                rapidjson::Document *d = new rapidjson::Document;
                d->SetObject();
                d->AddMember("opcode", "response_query", d->GetAllocator());
                rapidjson::Value str(rapidjson::kStringType);
                str.SetString(query_code.c_str(), d->GetAllocator());
                d->AddMember("type", str, d->GetAllocator());
                d->AddMember("data", "cannot found callback", d->GetAllocator());
                return d;
            }
        }

        void aerodynamics_configurer::init_query_functions()
        {
            query_functions["get_total_mass_inertial"] = [&](const rapidjson::Value &v) {
                rapidjson::Document *d = new rapidjson::Document;
                d->SetObject();
                d->AddMember("opcode", "response_query", d->GetAllocator());
                d->AddMember("type", "get_total_mass_inertial", d->GetAllocator());
                rapidjson::Value value(rapidjson::kObjectType);

                add_value(value, aircraftNode->get_total_mass(), *d, "total_mass");
                add_vector(value, aircraftNode->get_total_inertial(), *d, "total_inertial");

                d->AddMember("data", value, d->GetAllocator());
                return d;
            };
            query_functions["get_total_forces_torques"] = [&](const rapidjson::Value &v) {
                rapidjson::Document *d = new rapidjson::Document;
                d->SetObject();
                d->AddMember("opcode", "response_query", d->GetAllocator());
                d->AddMember("type", "get_total_forces_torques", d->GetAllocator());
                rapidjson::Value value(rapidjson::kObjectType);

                add_vector(value, aircraftNode->get_ground_transform().linear() * aircraftNode->get_total_force(), *d,
                           "total_force");
                add_vector(value, aircraftNode->get_ground_transform().linear() * aircraftNode->get_total_torque(), *d,
                           "total_torue");

                add_vector(value,
                           aircraftNode->get_ground_transform().linear() * aircraftNode->get_total_aerodynamics_force(),
                           *d, "total_airdynamics_force");
                add_vector(value, aircraftNode->get_ground_transform().linear() *
                                  aircraftNode->get_total_aerodynamics_torque(), *d, "total_airdynamics_torque");

                add_vector(value,
                           aircraftNode->get_ground_transform().linear() * aircraftNode->get_total_engine_torque(), *d,
                           "total_engine_torque");
                add_vector(value,
                           aircraftNode->get_ground_transform().linear() * aircraftNode->get_total_engine_force(), *d,
                           "total_engine_force");

                rapidjson::Value blade_array(rapidjson::kArrayType);
                blade_array.CopyFrom(
                       *aircraftNode->bladeElementManager.get_blades_information() ,
                        d->GetAllocator()
                );
                value.AddMember("blades",blade_array,d->GetAllocator());
                d->AddMember("data", value, d->GetAllocator());
                return d;
            };
            query_functions["get_internal_states_list"] = [&](const rapidjson::Value &v) {
                rapidjson::Document *d = new rapidjson::Document;
                d->SetObject();
                d->AddMember("opcode", "response_query", d->GetAllocator());
                d->AddMember("type", "get_internal_states_list", d->GetAllocator());
                rapidjson::Value value(rapidjson::kArrayType);

                for (auto pair : aircraftNode->get_internal_states()) {
                    rapidjson::Value v(rapidjson::kObjectType);
                    rapidjson::Value name(rapidjson::kStringType);
                    name.SetString(pair.first.c_str(), d->GetAllocator());
                    v.AddMember(name, pair.second, d->GetAllocator());
                    value.PushBack(v, d->GetAllocator());
                }

                d->AddMember("data", value, d->GetAllocator());
                return d;
            };
            query_functions["get_control_axis_list"] = [&](const rapidjson::Value &v) {
                rapidjson::Document *d = new rapidjson::Document;
                d->SetObject();
                d->AddMember("opcode", "response_query", d->GetAllocator());
                d->AddMember("type", "get_control_axis_list", d->GetAllocator());
                rapidjson::Value value(rapidjson::kArrayType);

                for (auto pair : aircraftNode->get_control_axis()) {
                    rapidjson::Value v(rapidjson::kObjectType);
                    rapidjson::Value name(rapidjson::kStringType);
                    name.SetString(pair.first.c_str(), d->GetAllocator());
                    v.AddMember(name, pair.second, d->GetAllocator());
                    value.PushBack(v, d->GetAllocator());
                }

                d->AddMember("data", value, d->GetAllocator());
                return d;
            };
            query_functions["set_control_axis_value"] = [&](const rapidjson::Value &v) {
                rapidjson::Document *d = new rapidjson::Document;
                d->SetObject();
                d->AddMember("opcode", "response_query", d->GetAllocator());
                d->AddMember("type", "set_control_axis_value", d->GetAllocator());

                std::string axis_name = fast_string(v, "control_axis_name");
                double axis_value = fast_value(v, "value");
                if (aircraftNode->set_control_value(axis_name, axis_value) == 0) {
                    d->AddMember("data", "successful", d->GetAllocator());
                }
                else {
                    d->AddMember("data", "failed", d->GetAllocator());
                }

                return d;
            };

            query_functions["set_internal_state_value"] = [&](const rapidjson::Value &v) {
                rapidjson::Document *d = new rapidjson::Document;
                d->SetObject();
                d->AddMember("opcode", "response_query", d->GetAllocator());
                d->AddMember("type", "set_internal_state_value", d->GetAllocator());

                std::string internal_name = fast_string(v, "internal_state_name");
                double internal_value = fast_value(v, "value");
                if (aircraftNode->set_internal_state(internal_name, internal_value) == 0) {
                    d->AddMember("data", "successful", d->GetAllocator());
                }
                else {
                    d->AddMember("data", "failed", d->GetAllocator());
                }

                return d;
            };

            query_functions["set_air_state"] = [&](const rapidjson::Value &v) {
                rapidjson::Document *d = new rapidjson::Document;
                d->SetObject();
                d->AddMember("opcode", "response_query", d->GetAllocator());
                d->AddMember("type", "set_air_state", d->GetAllocator());

                AirState airState;
                airState.rho = fast_value(v, "rho");
                airState.ground_air_speed = fast_vector3(v, "ground_air_speed");

                aircraftNode->set_air_state(airState);
                d->AddMember("data", "successful", d->GetAllocator());

                return d;
            };

            query_functions["set_flying_state"] = [&](const rapidjson::Value &v) {
                rapidjson::Document *d = new rapidjson::Document;
                d->SetObject();
                d->AddMember("opcode", "response_query", d->GetAllocator());
                d->AddMember("type", "set_flying_state", d->GetAllocator());

                ComponentData data;
                data.angular_velocity = fast_vector3(v, "angular_velocity");
                data.transform = fast_transform(v,"transform");
                data.body_transform = Eigen::Affine3d::Identity();
                data.ground_velocity = fast_vector3(v,"ground_velocity");
                aircraftNode->setStatefromsimulator(data,0);
                rapidjson::Value transdata(rapidjson::kObjectType);
                add_transform(transdata,aircraftNode->get_ground_transform(),*d);
                rapidjson::Value airspeed_value(rapidjson::kObjectType);

                AirState  airState =aircraftNode->airState;
                add_value(airspeed_value, data.get_airspeed_mag(airState), *d, "airspeed");
                add_value(airspeed_value, data.get_angle_of_attack(airState) * 180 / M_PI, *d, "angle_of_attack");
                add_value(airspeed_value, data.get_sideslip(airState), *d, "sideslip");
                transdata.AddMember("airstate", airspeed_value, d->GetAllocator());


                d->AddMember("data", transdata, d->GetAllocator());
                return d;
            };

            query_configure_functions["list_model"] = [&](const rapidjson::Value &v) {
                rapidjson::Document *d = new rapidjson::Document;
                d->SetObject();
                d->AddMember("opcode", "response_query_configurer", d->GetAllocator());
                d->AddMember("type", "list_model", d->GetAllocator());
                rapidjson::Value value(rapidjson::kArrayType);

                for (auto model : list_model()) {
                    rapidjson::Value name(rapidjson::kStringType);
                    name.SetString(model.c_str(), d->GetAllocator());
                    value.PushBack(name, d->GetAllocator());
                }

                d->AddMember("data", value, d->GetAllocator());
                return d;
            };

            query_configure_functions["load_model"] = [&](const rapidjson::Value &v) {
                rapidjson::Document *d = new rapidjson::Document;
                d->SetObject();

                d->AddMember("opcode", "response_query_configurer", d->GetAllocator());
                d->AddMember("type", "load_model", d->GetAllocator());

                std::string model_name = v["name"].GetString();
                rapidjson::Value data_field(rapidjson::kStringType);
                data_field.SetString("data", d->GetAllocator());
                rapidjson::Value value(rapidjson::kObjectType);
                value.CopyFrom(load_model(model_name), d->GetAllocator());
                d->AddMember(data_field, value, d->GetAllocator());
                return d;
            };

            query_configure_functions["chroot_folder"] = [&](const rapidjson::Value &v) {
                rapidjson::Document *d = new rapidjson::Document;
                d->SetObject();

                d->AddMember("opcode", "response_query_configurer", d->GetAllocator());
                d->AddMember("type", "chroot_folder", d->GetAllocator());
                rapidjson::Value value(rapidjson::kObjectType);

                d->AddMember("data", value, d->GetAllocator());
                return d;
            };

//            query_configure_functions[""]
        }
    }
}